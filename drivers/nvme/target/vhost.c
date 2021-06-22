#include <linux/module.h>
#include <linux/compat.h>
#include <linux/eventfd.h>
#include <linux/vhost.h>
#include <linux/vhost_types.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/file.h>
#include <linux/highmem.h>
#include <linux/kthread.h>
#include "../../vhost/vhost.h"
#include "nvmet.h"

#define NVMET_VHOST_AQ_DEPTH		256
#define NVMET_VHOST_MAX_SEGMENTS	32

enum NvmeCcShift {
	CC_MPS_SHIFT	= 7,
	CC_IOSQES_SHIFT	= 16,
	CC_IOCQES_SHIFT	= 20,
};

enum NvmeCcMask {
	CC_MPS_MASK	= 0xf,
	CC_IOSQES_MASK	= 0xf,
	CC_IOCQES_MASK	= 0xf,
};

#define NVME_CC_MPS(cc)    ((cc >> CC_MPS_SHIFT)    & CC_MPS_MASK)
#define NVME_CC_IOSQES(cc) ((cc >> CC_IOSQES_SHIFT) & CC_IOSQES_MASK)
#define NVME_CC_IOCQES(cc) ((cc >> CC_IOCQES_SHIFT) & CC_IOCQES_MASK)

enum NvmeAqaShift {
	AQA_ASQS_SHIFT	= 0,
	AQA_ACQS_SHIFT	= 16,
};

enum NvmeAqaMask {
	AQA_ASQS_MASK	= 0xfff,
	AQA_ACQS_MASK	= 0xfff,
};

#define NVME_AQA_ASQS(aqa) ((aqa >> AQA_ASQS_SHIFT) & AQA_ASQS_MASK)
#define NVME_AQA_ACQS(aqa) ((aqa >> AQA_ACQS_SHIFT) & AQA_ACQS_MASK)

#define NVME_CQ_FLAGS_PC(cq_flags)	(cq_flags & 0x1)
#define NVME_CQ_FLAGS_IEN(cq_flags)	((cq_flags >> 1) & 0x1)

#define NVME_SQ_FLAGS_PC(sq_flags)	(sq_flags & 0x1)

struct nvmet_vhost_ctrl_eventfd {
	struct file *call;
	struct eventfd_ctx *call_ctx;
	int __user *irq_enabled;
	int __user *vector;
};

struct nvmet_vhost_iod {
	struct nvmet_vhost_sq	*sq;
	struct scatterlist	sg[NVMET_VHOST_MAX_SEGMENTS];
	struct nvme_command	cmd;
	struct nvme_completion	rsp;
	struct nvmet_req	req;
	struct list_head	entry;
};

struct nvmet_vhost_cq {
	struct nvmet_cq		cq;
	struct nvmet_vhost_ctrl	*ctrl;

	u32			head;
	u32			tail;
	u8			phase;
	u64			dma_addr;
	struct eventfd_ctx	*eventfd;

	struct list_head	sq_list;
	struct list_head	req_list;
	spinlock_t		lock;
	struct task_struct	*thread;
	int			scheduled;
};

struct nvmet_vhost_sq {
	struct nvmet_sq		sq;
	struct nvmet_vhost_ctrl	*ctrl;

	u32			head;
	u32			tail;
	u64			dma_addr;
	u16			cqid;

	struct nvmet_vhost_iod	*io_req;
	struct list_head	req_list;
	struct list_head	entry;
	struct mutex            lock;
	struct task_struct	*thread;
	int			scheduled;
};

struct nvmet_vhost_ctrl {
	struct vhost_dev vdev;
	struct nvmet_vhost_ctrl_eventfd *eventfd;

	u16 cntlid;
	struct nvmet_ctrl *ctrl;
	u32 num_queues;

	struct nvmet_vhost_cq **cqs;
	struct nvmet_vhost_sq **sqs;
	struct nvmet_vhost_cq admin_cq;
	struct nvmet_vhost_sq admin_sq;

	u32 aqa;
	u64 asq;
	u64 acq;
	u16 cqe_size;
	u16 sqe_size;
	u16 max_prp_ents;
	u16 page_bits;
	u32 page_size;
};

static int nvmet_vhost_read(struct vhost_dev *vdev, u64 guest_pa,
		void *buf, uint32_t size)
{
	return vhost_mem_copy_to_user(vdev, buf, (void *)guest_pa, size);
}

static int nvmet_vhost_write(struct vhost_dev *vdev, u64 guest_pa,
		void *buf, uint32_t size)
{
	return vhost_mem_copy_from_user(vdev, buf, (void *)guest_pa, size);
}

#define sq_to_vsq(sq) container_of(sq, struct nvmet_vhost_sq, sq)
#define cq_to_vcq(cq) container_of(cq, struct nvmet_vhost_cq, cq)

static int nvmet_vhost_check_sqid(struct nvmet_ctrl *ctrl, u16 sqid)
{
	return sqid <= ctrl->subsys->max_qid && ctrl->sqs[sqid] != NULL ? 0 : -1;
}

static int nvmet_vhost_check_cqid(struct nvmet_ctrl *ctrl, u16 cqid)
{
	return cqid <= ctrl->subsys->max_qid && ctrl->cqs[cqid] != NULL ? 0 : -1;
}

static void nvmet_vhost_inc_cq_tail(struct nvmet_vhost_cq *cq)
{
	cq->tail++;
	if (cq->tail >= cq->cq.size) {
		cq->tail = 0;
		cq->phase = !cq->phase;
	}
}

static void nvmet_vhost_inc_sq_head(struct nvmet_vhost_sq *sq)
{
	sq->head = (sq->head + 1) % sq->sq.size;
}

static uint8_t nvmet_vhost_cq_full(struct nvmet_vhost_cq *cq)
{
	return (cq->tail + 1) % cq->cq.size == cq->head;
}

static uint8_t nvmet_vhost_sq_empty(struct nvmet_vhost_sq *sq)
{
	return sq->head == sq->tail;
}

static void nvmet_vhost_post_cqes(struct nvmet_vhost_cq *cq)
{
	struct nvmet_vhost_ctrl *n = cq->ctrl;
	struct nvmet_vhost_iod *req;
	struct list_head *p, *tmp;
	int signal = 0;
	unsigned long flags;

	spin_lock_irqsave(&cq->lock, flags);
	list_for_each_safe(p, tmp, &cq->req_list) {
		struct nvmet_vhost_sq *sq;
		u64 addr;

		if (nvmet_vhost_cq_full(cq))
			goto unlock;

		req = list_entry(p, struct nvmet_vhost_iod, entry);
		list_del(p);

		sq = req->sq;
		req->rsp.status |= cq->phase;
		req->rsp.sq_id = cpu_to_le16(sq->sq.qid);
		req->rsp.sq_head = cpu_to_le16(sq->head);
		addr = cq->dma_addr + cq->tail * n->cqe_size;
		nvmet_vhost_inc_cq_tail(cq);
		spin_unlock_irqrestore(&cq->lock, flags);

		nvmet_vhost_write(&n->vdev, addr, (void *)&req->rsp,
			sizeof(req->rsp));

		mutex_lock(&sq->lock);
		list_add_tail(p, &sq->req_list);
		mutex_unlock(&sq->lock);

		signal = 1;

		spin_lock_irqsave(&cq->lock, flags);
	}

	if (signal)
		eventfd_signal(cq->eventfd, 1);

unlock:
	cq->scheduled = 0;
	spin_unlock_irqrestore(&cq->lock, flags);
}

static int nvmet_vhost_cq_thread(void *arg)
{
	struct nvmet_vhost_cq *sq = arg;

	while (!kthread_should_stop()) {
		nvmet_vhost_post_cqes(sq);

		schedule();
	}

	return 0;
}

static void nvmet_vhost_enqueue_req_completion(
		struct nvmet_vhost_cq *cq, struct nvmet_vhost_iod *iod)
{
	unsigned long flags;

	BUG_ON(cq->cq.qid != iod->sq->sq.qid);
	spin_lock_irqsave(&cq->lock, flags);
	list_add_tail(&iod->entry, &cq->req_list);
	if (!cq->scheduled) {
		wake_up_process(cq->thread);
		cq->scheduled = 1;
	}
	spin_unlock_irqrestore(&cq->lock, flags);
}

__maybe_unused
static void nvmet_vhost_queue_response(struct nvmet_req *req)
{
	struct nvmet_vhost_iod *iod =
		container_of(req, struct nvmet_vhost_iod, req);
	struct nvmet_vhost_sq *sq = iod->sq;
	struct nvmet_vhost_ctrl *n = sq->ctrl;
	struct nvmet_vhost_cq *cq = n->cqs[sq->sq.qid];

	nvmet_vhost_enqueue_req_completion(cq, iod);
}

static int nvmet_vhost_sglist_add(struct nvmet_vhost_ctrl *ctrl, struct scatterlist *sg,
		u64 guest_addr, int len, int is_write)
{
	void __user *host_addr;
	struct page *page;
	unsigned int offset, nbytes;
	int ret;

#if 0
	host_addr = map_guest_to_host(&ctrl->dev, guest_addr, len);
#endif
	if (unlikely(!host_addr)) {
		pr_warn("cannot map guest addr %p, error %ld\n",
			(void *)guest_addr, PTR_ERR(host_addr));
		return PTR_ERR(host_addr);
	}

	ret = get_user_pages((unsigned long)host_addr, 1,
			     0, &page, NULL);
	BUG_ON(ret == 0); /* we should either get our page or fail */
	if (ret < 0) {
		pr_warn("get_user_pages faild: host_addr %p, %d\n",
			host_addr, ret);
		return ret;
	}

	offset = (uintptr_t)host_addr & ~PAGE_MASK;
	nbytes = min_t(unsigned int, PAGE_SIZE - offset, len);
	sg_set_page(sg, page, nbytes, offset);

	return 0;
}

static int nvmet_vhost_map_prp(struct nvmet_vhost_ctrl *ctrl, struct scatterlist *sgl,
			       u64 prp1, u64 prp2, unsigned int len)
{
	unsigned int trans_len = ctrl->page_size - (prp1 % ctrl->page_size);
	int num_prps = (len >> ctrl->page_bits) + 1;
	//FIXME
	int is_write = 1;

	trans_len = min(len, trans_len);
	if (!prp1)
		return -1;

	sg_init_table(sgl, num_prps);

	nvmet_vhost_sglist_add(ctrl, sgl, prp1, trans_len, is_write);

	len -= trans_len;
	if (len) {
		if (!prp2)
			goto error;
		if (len > ctrl->page_size) {
			u64 *prp_list = kcalloc(ctrl->max_prp_ents, sizeof(u64), GFP_KERNEL);
			u16 nents, prp_trans;
			int i = 0;

			nents = (len + ctrl->page_size - 1) >> ctrl->page_bits;
			prp_trans = min(ctrl->max_prp_ents, nents) * sizeof(u64);
			nvmet_vhost_read(&ctrl->vdev, prp2, (void *)prp_list, prp_trans);

			while (len != 0) {
				u64 prp_ent = le64_to_cpu(prp_list[i]);

				if (i == ctrl->max_prp_ents - 1 && len > ctrl->page_size) {
					if (!prp_ent || prp_ent & (ctrl->page_size - 1)) {
						kfree(prp_list);
						goto error;
					}
					i = 0;
					nents = (len + ctrl->page_size - 1) >> ctrl->page_bits;
					prp_trans = min(ctrl->max_prp_ents, nents) * sizeof(u64);
					nvmet_vhost_read(&ctrl->vdev, prp_ent, (void *)prp_list, prp_trans);
					prp_ent = le64_to_cpu(prp_list[i]);
				}

				if (!prp_ent || prp_ent & (ctrl->page_size - 1)) {
					kfree(prp_list);
					goto error;
				}

				trans_len = min(len, ctrl->page_size);
				nvmet_vhost_sglist_add(ctrl, sgl, prp_ent, trans_len, is_write);
				sgl++;
				len -= trans_len;
				i++;
			}
			kfree(prp_list);
		} else {
			if (prp2 & (ctrl->page_size - 1))
				goto error;
			nvmet_vhost_sglist_add(ctrl, sgl, prp2, trans_len, is_write);
		}
	}

	return num_prps;

error:
	return -1;
}


//TODO: Implement properly
struct nvmet_fabrics_ops nvmet_vhost_ops = {
};

static void nvmet_vhost_process_sq(struct nvmet_vhost_sq *sq)
{
	struct nvmet_vhost_ctrl *ctrl = sq->ctrl;
	struct nvmet_vhost_cq *cq = ctrl->cqs[sq->sq.qid];
	struct nvmet_vhost_iod *iod;
	struct nvme_command *cmd;
	int ret;

	mutex_lock(&sq->lock);

	while (!(nvmet_vhost_sq_empty(sq) || list_empty(&sq->req_list))) {
		u64 addr = sq->dma_addr + sq->head * ctrl->sqe_size;;

		nvmet_vhost_inc_sq_head(sq);
		iod = list_first_entry(&sq->req_list,
					struct nvmet_vhost_iod, entry);
		list_del(&iod->entry);
		mutex_unlock(&sq->lock);

		cmd = &iod->cmd;
		ret = nvmet_vhost_read(&ctrl->vdev, addr,
				(void *)cmd, sizeof(*cmd));
		if (ret) {
			pr_warn("nvmet_vhost_read fail\n");
			goto out;
		}

		ret = nvmet_req_init(&iod->req, &cq->cq, &sq->sq, &nvmet_vhost_ops);
		if (ret) {
			pr_warn("nvmet_req_init error: ret 0x%x, qid %d\n", ret, sq->sq.qid);
			goto out;
		}
		if (iod->req.transfer_len) {
			ret = nvmet_vhost_map_prp(ctrl, iod->sg, cmd->common.dptr.prp1,
						  cmd->common.dptr.prp2,
						  iod->req.transfer_len);
			if (ret > 0) {
				iod->req.sg = iod->sg;
				iod->req.sg_cnt = ret;
			} else {
				pr_warn("map prp error\n");
				goto out;
			}
		}
		iod->req.execute(&iod->req);
		mutex_lock(&sq->lock);
        }

unlock:
	sq->scheduled = 0;
	mutex_unlock(&sq->lock);
	return;

out:
	mutex_lock(&sq->lock);
	list_add_tail(&iod->entry, &sq->req_list);
	goto unlock;
}

static int nvmet_vhost_sq_thread(void *opaque)
{
	struct nvmet_vhost_sq *sq = opaque;

	while (!kthread_should_stop()) {
		nvmet_vhost_process_sq(sq);

		schedule();
	}

	return 0;
}

static int nvmet_vhost_init_cq(struct nvmet_vhost_cq *cq,
			       struct nvmet_vhost_ctrl *ctrl, u64 dma_addr,
			       u16 cqid, u16 size, struct eventfd_ctx *eventfd,
			       u16 vector, u16 irq_enabled)
{
	cq->ctrl = ctrl;
	cq->dma_addr = dma_addr;
	cq->phase = 1;
	cq->head = cq->tail = 0;
	cq->eventfd = eventfd;
	ctrl->cqs[cqid] = cq;

	spin_lock_init(&cq->lock);
	INIT_LIST_HEAD(&cq->req_list);
	INIT_LIST_HEAD(&cq->sq_list);
	cq->scheduled = 0;
	cq->thread = kthread_create(nvmet_vhost_cq_thread, cq, "nvmet_vhost_cq");

	nvmet_cq_setup(ctrl->ctrl, &cq->cq, cqid, size);

	return 0;
}

static int nvmet_vhost_init_sq(struct nvmet_vhost_sq *sq,
			       struct nvmet_vhost_ctrl *ctrl, u64 dma_addr,
			       u16 sqid, u16 cqid, u16 size)
{
	struct nvmet_vhost_cq *cq;
	struct nvmet_vhost_iod *iod;
	int i;

	sq->ctrl = ctrl;
	sq->dma_addr = dma_addr;
	sq->cqid = cqid;
	sq->head = sq->tail = 0;
	ctrl->sqs[sqid] = sq;

	mutex_init(&sq->lock);
	INIT_LIST_HEAD(&sq->req_list);
	sq->io_req = kmalloc(sizeof(struct nvmet_vhost_iod) * size, GFP_KERNEL);
	if (!sq->io_req)
		return -ENOMEM;
	for (i = 0; i < size; i++) {
		iod = &sq->io_req[i];

		iod->req.cmd = &iod->cmd;
		/* iod->req.rsp = &iod->rsp; */
		iod->sq = sq;
		list_add_tail(&iod->entry, &sq->req_list);
	}
	sq->scheduled = 0;
	sq->thread = kthread_create(nvmet_vhost_sq_thread, sq, "nvmet_vhost_sq");

	cq = ctrl->cqs[cqid];
	list_add_tail(&sq->entry, &cq->sq_list);
	ctrl->sqs[sqid] = sq;

	nvmet_sq_setup(ctrl->ctrl, &sq->sq, sqid, size);

	return 0;
}

__maybe_unused
static void nvmet_vhost_start_ctrl(void *priv)
{
	struct nvmet_vhost_ctrl *ctrl = priv;
	u32 page_bits = NVME_CC_MPS(ctrl->ctrl->cc) + 12;
	u32 page_size = 1 << page_bits;
	int ret;

	ctrl->page_bits = page_bits;
	ctrl->page_size = page_size;
	ctrl->max_prp_ents = ctrl->page_size / sizeof(uint64_t);
	ctrl->cqe_size = 1 << NVME_CC_IOCQES(ctrl->ctrl->cc);
	ctrl->sqe_size = 1 << NVME_CC_IOSQES(ctrl->ctrl->cc);

	nvmet_vhost_init_cq(&ctrl->admin_cq, ctrl, ctrl->acq, 0,
		NVME_AQA_ACQS(ctrl->aqa) + 1, ctrl->eventfd[0].call_ctx,
		0, 1);

	ret = nvmet_vhost_init_sq(&ctrl->admin_sq, ctrl, ctrl->asq, 0, 0,
		NVME_AQA_ASQS(ctrl->aqa) + 1);
	if (ret) {
		pr_warn("nvmet_vhost_init_sq failed!!!\n");
		BUG_ON(1);
	}
}

static void nvmet_vhost_create_cq(struct nvmet_req *req)
{
	struct nvmet_cq *cq;
	struct nvmet_vhost_cq *vcq;
	struct nvmet_vhost_ctrl *ctrl;
	struct nvme_create_cq *cmd;
	u16 cqid;
	u16 vector;
	u16 qsize;
	u16 qflags;
	u64 prp1;
	int status;
	int ret;

	cq = req->cq;
	vcq = cq_to_vcq(cq);
	ctrl = vcq->ctrl;
	cmd = &req->cmd->create_cq;
	cqid = le16_to_cpu(cmd->cqid);
	vector = le16_to_cpu(cmd->irq_vector);
	qsize = le16_to_cpu(cmd->qsize);
	qflags = le16_to_cpu(cmd->cq_flags);
	prp1 = le64_to_cpu(cmd->prp1);
	status = NVME_SC_SUCCESS;

	if (!cqid || (cqid && !nvmet_vhost_check_cqid(ctrl->ctrl, cqid))) {
		status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		goto out;
	}
	if (!qsize || qsize > NVME_CAP_MQES(ctrl->ctrl->cap)) {
		status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		goto out;
	}
	if (!prp1) {
		status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
		goto out;
	}
	if (vector > ctrl->num_queues) {
		status = NVME_SC_INVALID_VECTOR | NVME_SC_DNR;
		goto out;
	}
	if (!(NVME_CQ_FLAGS_PC(qflags))) {
		status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
		goto out;
	}

	vcq = kmalloc(sizeof(*vcq), GFP_KERNEL);
	if (!vcq) {
		status = NVME_SC_INTERNAL | NVME_SC_DNR;
		goto out;
	}

	ret = nvmet_vhost_init_cq(vcq, ctrl, prp1, cqid, qsize+1,
		ctrl->eventfd[cqid].call_ctx, vector,
		NVME_CQ_FLAGS_IEN(qflags));
	if (ret)
		status = NVME_SC_INTERNAL | NVME_SC_DNR;

out:
	nvmet_req_complete(req, status);
}

static void nvmet_vhost_create_sq(struct nvmet_req *req)
{
	struct nvme_create_sq *cmd = &req->cmd->create_sq;
	u16 cqid = le16_to_cpu(cmd->cqid);
	u16 sqid = le16_to_cpu(cmd->sqid);
	u16 qsize = le16_to_cpu(cmd->qsize);
	u16 qflags = le16_to_cpu(cmd->sq_flags);
	u64 prp1 = le64_to_cpu(cmd->prp1);

	struct nvmet_sq *sq = req->sq;
	struct nvmet_vhost_sq *vsq;
	struct nvmet_vhost_ctrl *ctrl;
	int status;
	int ret;

	status = NVME_SC_SUCCESS;
	vsq = sq_to_vsq(sq);
	ctrl = vsq->ctrl;

	if (!cqid || nvmet_vhost_check_cqid(ctrl->ctrl, cqid)) {
		status = NVME_SC_CQ_INVALID | NVME_SC_DNR;
		goto out;
	}
	if (!sqid || (sqid && !nvmet_vhost_check_sqid(ctrl->ctrl, sqid))) {
		status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		goto out;
	}
	if (!qsize || qsize > NVME_CAP_MQES(ctrl->ctrl->cap)) {
		status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		goto out;
	}
	if (!prp1 || prp1 & (ctrl->page_size - 1)) {
		status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
		goto out;
	}
	if (!(NVME_SQ_FLAGS_PC(qflags))) {
		status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
		goto out;
	}

	vsq = kmalloc(sizeof(*vsq), GFP_KERNEL);
	if (!sq) {
		status = NVME_SC_INTERNAL | NVME_SC_DNR;
		goto out;
	}

	ret = nvmet_vhost_init_sq(vsq, ctrl, prp1, sqid, cqid, qsize + 1);
	if (ret)
		status = NVME_SC_INTERNAL | NVME_SC_DNR;

out:
	nvmet_req_complete(req, status);
}

__maybe_unused
static int nvmet_vhost_process_iotlb_msg(struct vhost_dev *dev,
					 struct vhost_iotlb_msg *msg)
{
	/* struct nvmet_vhost_ctrl *ctrl = container_of(dev, struct nvmet_vhost_ctrl, vdev); */
	//TODO: Determine if any IOTLB messages need to be processed
	return 0;
}


__maybe_unused
static int nvmet_vhost_parse_admin_cmd(struct nvmet_req *req)
{
	struct nvme_command *cmd = req->cmd;

	switch (cmd->common.opcode) {
	case nvme_admin_create_cq:
		req->execute = nvmet_vhost_create_cq;
		req->transfer_len = 0;
		return 0;
	case nvme_admin_create_sq:
		req->execute = nvmet_vhost_create_sq;
		req->transfer_len = 0;
		return 0;
	}

	return -1;
}

static int
nvmet_vhost_set_endpoint(struct nvmet_vhost_ctrl *ctrl,
			 struct vhost_nvme_target *nvme_tgt)
{
	int num_queues;
	int ret = 0;

	//TODO: Determine if any locking is needed
	if (IS_ERR(ctrl)) {
	  	pr_err("Pointer to ctrl is error %ld\n", PTR_ERR(ctrl));
		return -EINVAL;
	}
	
	pr_warn("Set endpoint start");
	ctrl->num_queues = 1;
	num_queues = ctrl->num_queues;

	ctrl->cqs = kzalloc(sizeof(struct nvme_vhost_cq *) * num_queues, GFP_KERNEL);
	if (!ctrl->cqs) {
		ret = -ENOMEM;
		goto out_ctrl_put;
	}

	ctrl->sqs = kzalloc(sizeof(struct nvme_vhost_sq *) * num_queues, GFP_KERNEL);
	if (!ctrl->sqs) {
		ret = -ENOMEM;
		goto free_cqs;
	}

	ctrl->eventfd = kmalloc(sizeof(struct nvmet_vhost_ctrl_eventfd) * num_queues,
				GFP_KERNEL);
	if (!ctrl->eventfd) {
		ret = -ENOMEM;
		goto free_sqs;
	}
	pr_warn("Set endpoint end");
	return 0;

free_sqs:
	kfree(ctrl->sqs);
free_cqs:
	kfree(ctrl->cqs);
out_ctrl_put:
	pr_warn("Set endpoint end with err");
	return ret;
}

static int
nvmet_vhost_clear_endpoint(struct nvmet_vhost_ctrl *ctrl,
			   struct vhost_nvme_target *nvme_tgt)
{
	struct nvmet_ctrl *tgt_ctrl;

	//TODO: Determine if any locking is needed
	if (IS_ERR(ctrl)) {
		return -EINVAL;
	}

	tgt_ctrl = ctrl->ctrl;

	if (IS_ERR(tgt_ctrl)) {
		return -EINVAL;
	}

	//TODO: Check if the controller is stopped

	if (ctrl->sqs) {
		kfree(ctrl->sqs);
		ctrl->sqs = NULL;
	}
	if (ctrl->cqs) {
		kfree(ctrl->cqs);
		ctrl->cqs = NULL;
	}
	if (ctrl->eventfd) {
		kfree(ctrl->eventfd);
		ctrl->eventfd = NULL;
	}

	nvmet_ctrl_put(tgt_ctrl);

	return 0;
}

static int nvmet_vhost_set_eventfd(struct nvmet_vhost_ctrl *ctrl, void __user *argp)
{
	struct nvmet_vhost_eventfd eventfd;
	int num;
	int ret;

	if (IS_ERR(ctrl)) {
		return -EINVAL;
	}

	ret = copy_from_user(&eventfd, argp, sizeof(struct nvmet_vhost_eventfd));
	if (unlikely(ret))
		return ret;

	num = eventfd.num;
	if (num > ctrl->ctrl->subsys->max_qid)
		return -EINVAL;

	pr_err("Set eventfd is not implemented");

	/*ctrl->eventfd[num].call = eventfd_fget(eventfd.fd);
	if (IS_ERR(ctrl->eventfd[num].call))
		return -EBADF;
	ctrl->eventfd[num].call_ctx = eventfd_ctx_fileget(ctrl->eventfd[num].call);
	if (IS_ERR(ctrl->eventfd[num].call_ctx)) {
		fput(ctrl->eventfd[num].call);
		return -EBADF;
	}

	ctrl->eventfd[num].irq_enabled = eventfd.irq_enabled;
	ctrl->eventfd[num].vector = eventfd.vector;
	*/
	return 0;
}

static int nvmet_vhost_bar_read(struct nvmet_ctrl *ctrl, int offset, u64 *val)
{
	int status = NVME_SC_SUCCESS;

	switch(offset) {
	case NVME_REG_CAP:
		*val = ctrl->cap;
		break;
	case NVME_REG_CAP + 4:
		*val = ctrl->cap >> 32;
		break;
	case NVME_REG_VS:
		*val = ctrl->subsys->ver;
		break;
	case NVME_REG_CC:
		*val = ctrl->cc;
		break;
	case NVME_REG_CSTS:
		*val = ctrl->csts;
		break;
	case NVME_REG_AQA:
		*val = (NVMET_VHOST_AQ_DEPTH - 1) |
		      (((NVMET_VHOST_AQ_DEPTH - 1) << 16));
		break;
	default:
		printk("Unknown offset: 0x%x\n", offset);
		status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
		break;
	}

	return status;
}

//TODO: Reuse the generic bar_write function
static int nvmet_bar_write(struct nvmet_vhost_ctrl *ctrl, int offset, u64 val)
{
	/* struct nvmet_ctrl *nvme_ctrl = ctrl->ctrl; */
	int status = NVME_SC_SUCCESS;

	switch(offset) {
//	case NVME_REG_CC:
//		nvmet_update_cc(nvme_ctrl, val);
		break;
	case NVME_REG_AQA:
		ctrl->aqa = val & 0xffffffff;
		break;
	case NVME_REG_ASQ:
		ctrl->asq = val;
		break;
	case NVME_REG_ASQ + 4:
		ctrl->asq |= val << 32;
		break;
	case NVME_REG_ACQ:
		ctrl->acq = val;
		break;
	case NVME_REG_ACQ + 4:
		ctrl->acq |= val << 32;
		break;
	default:
		status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
		break;
	}

	return status;
}

static int nvmet_vhost_process_db(struct nvmet_ctrl *ctrl, int offset, u64 val)
{
	u16 qid;

	if (offset & ((1 << 2) - 1))
		return -EINVAL;

	if (((offset - 0x1000) >> 2) & 1) {
		u16 new_head = val & 0xffff;
		int start_sqs;
		struct nvmet_vhost_cq *vcq;
		struct nvmet_cq *cq;
		unsigned long flags;

		qid = (offset - (0x1000 + (1 << 2))) >> 3;
		if (nvmet_vhost_check_cqid(ctrl, qid))
			return -EINVAL;

		cq = ctrl->cqs[qid];
		if (new_head >= cq->size)
			return -EINVAL;

		vcq = cq_to_vcq(cq);
		spin_lock_irqsave(&vcq->lock, flags);
		start_sqs = nvmet_vhost_cq_full(vcq) ? 1 : 0;
		vcq->head = new_head;
		spin_unlock_irqrestore(&vcq->lock, flags);
		if (start_sqs) {
			struct nvmet_vhost_sq *sq;
			struct list_head *p;

			list_for_each(p, &vcq->sq_list) {
				sq = list_entry(p, struct nvmet_vhost_sq, entry);
				if (!sq->scheduled) {
					sq->scheduled = 1;
					wake_up_process(sq->thread);
				}
			}
			if (!vcq->scheduled) {
				vcq->scheduled = 1;
				wake_up_process(vcq->thread);
			}
		}

		if (vcq->tail != vcq->head)
			eventfd_signal(vcq->eventfd, 1);
	} else {
		struct nvmet_vhost_sq *vsq;
		struct nvmet_sq *sq;
		u16 new_tail = val & 0xffff;

		qid = (offset - 0x1000) >> 3;
		if (nvmet_vhost_check_sqid(ctrl, qid))
			return -EINVAL;

		sq = ctrl->sqs[qid];
		if (new_tail >= sq->size)
			return -ENOSPC;

		vsq = sq_to_vsq(sq);
		mutex_lock(&vsq->lock);
		vsq->tail = new_tail;
		if (!vsq->scheduled) {
			vsq->scheduled = 1;
			wake_up_process(vsq->thread);
		}
		mutex_unlock(&vsq->lock);
	}

	return 0;
}

static int nvmet_vhost_bar_write(struct nvmet_vhost_ctrl *ctrl, int offset, u64 val)
{
	if (offset < 0x1000)
		return nvmet_bar_write(ctrl, offset, val);

	return nvmet_vhost_process_db(ctrl->ctrl, offset, val);
}

static int nvmet_vhost_ioc_bar(struct nvmet_vhost_ctrl *ctrl, void __user *argp)
{
	struct nvmet_vhost_bar bar;
	struct nvmet_vhost_bar __user *user_bar = argp;
	int ret = -EINVAL;

	ret = copy_from_user(&bar, argp, sizeof(bar));
	if (unlikely(ret))
		return ret;

	pr_warn("nvmet_vhost_ioc_bar ioctl");
	if (bar.type == VHOST_NVME_BAR_READ) {
		u64 val;
		ret = nvmet_vhost_bar_read(ctrl->ctrl, bar.offset, &val);
		if (ret != NVME_SC_SUCCESS)
			return ret;
		ret = copy_to_user(&user_bar->val, &val, sizeof(u64));
	} else if (bar.type == VHOST_NVME_BAR_WRITE)
		ret = nvmet_vhost_bar_write(ctrl, bar.offset, bar.val);

	return ret;
}

__maybe_unused
static int nvmet_vhost_open(struct inode *inode, struct file *f)
{
	struct nvmet_vhost_ctrl *ctrl = kzalloc(sizeof(struct nvmet_vhost_ctrl),
						GFP_KERNEL);

	if (!ctrl)
		return -ENOMEM;

	/* We don't use virtqueue */
	pr_warn("vhost_dev init in kernel");
	vhost_dev_init(&ctrl->vdev, NULL, 0, 0, 0, 0,
		       false, nvmet_vhost_process_iotlb_msg);

	f->private_data = ctrl;

	return 0;
}

static void nvme_free_sq(struct nvmet_vhost_sq *sq,
		struct nvmet_vhost_ctrl *ctrl)
{
	ctrl->sqs[sq->sq.qid] = NULL;
	kthread_stop(sq->thread);
	kfree(sq->io_req);
	if (sq->sq.qid)
		kfree(sq);
}

static void nvme_free_cq(struct nvmet_vhost_cq *cq,
			 struct nvmet_vhost_ctrl *ctrl)
{
	ctrl->cqs[cq->cq.qid] = NULL;
	kthread_stop(cq->thread);
	if (cq->cq.qid)
		kfree(cq);
}

__maybe_unused
static void nvmet_vhost_clear_ctrl(struct nvmet_vhost_ctrl *ctrl)
{
	int i;

	for (i = 0; i < ctrl->num_queues; i++) {
		if (ctrl->sqs[i] != NULL)
			nvme_free_sq(ctrl->sqs[i], ctrl);
	}
	for (i = 0; i < ctrl->num_queues; i++) {
		if (ctrl->cqs[i] != NULL)
			nvme_free_cq(ctrl->cqs[i], ctrl);
	}

	kfree(ctrl->eventfd);
	kfree(ctrl->cqs);
	kfree(ctrl->sqs);
	nvmet_ctrl_put(ctrl->ctrl);
}

__maybe_unused
static void nvmet_vhost_clear_eventfd(struct nvmet_vhost_ctrl *ctrl)
{
	int i;

	for (i = 0; i < ctrl->num_queues; i++) {
		if (ctrl->eventfd[i].call_ctx) {
			eventfd_ctx_put(ctrl->eventfd[i].call_ctx);
			fput(ctrl->eventfd[i].call);
		}
	}
}

static int nvmet_vhost_release(struct inode *inode, struct file *f)
{
	struct nvmet_vhost_ctrl *ctrl = f->private_data;

	if (IS_ERR(ctrl)) {
	  pr_err("release: Pointer to ctrl is error %ld\n", PTR_ERR(ctrl));
		return -EINVAL;
	}

	// nvmet_vhost_clear_eventfd(ctrl);
	// nvmet_vhost_clear_ctrl(ctrl);

	// vhost_dev_stop(&ctrl->vdev);
	// vhost_dev_cleanup(&ctrl->vdev);

	kfree(ctrl);
	return 0;
}

static int vhost_nvme_set_features(struct nvmet_vhost_ctrl *ctrl, u64 features)
{
  pr_err("Set features is not implemented");
  return 0;
}

static long nvmet_vhost_ioctl(struct file *f, unsigned int ioctl,
			      unsigned long arg)
{
	struct nvmet_vhost_ctrl *ctrl = f->private_data;
	struct vhost_nvme_target conf;
	void __user *argp = (void __user *)arg;
	u64 features;
	int r;

    if (IS_ERR(ctrl)) {
	  pr_err("Pointer to ctrl is error %ld\n", PTR_ERR(ctrl));
		return -EINVAL;
	}

	switch (ioctl) {
	case VHOST_NVME_SET_ENDPOINT:
		pr_warn("VHOST_NVME_SET_ENDPOINT");
		if (copy_from_user(&conf, argp, sizeof(conf)))
			return -EFAULT;

		return nvmet_vhost_set_endpoint(ctrl, &conf);
	case VHOST_NVME_CLEAR_ENDPOINT:
		pr_warn("VHOST_NVME_CLEAR_ENDPOINT");
		if (copy_from_user(&conf, argp, sizeof(conf)))
			return -EFAULT;

		return nvmet_vhost_clear_endpoint(ctrl, &conf);
	case VHOST_NVME_SET_EVENTFD:
		pr_warn("VHOST_NVME_SET_EVENTFD");
		r = nvmet_vhost_set_eventfd(ctrl, argp);
		return r;
	case VHOST_NVME_BAR:
		pr_warn("VHOST_NVME_BAR");
		return nvmet_vhost_ioc_bar(ctrl, argp);
	case VHOST_GET_FEATURES:
		pr_warn("VHOST_GET_FEATURES");
		features = VHOST_FEATURES;
		if (copy_to_user(argp, &features, sizeof(features)))
			return -EFAULT;
		return 0;
	case VHOST_SET_FEATURES:
		pr_warn("VHOST_SET_FEATURES");
		if (copy_from_user(&features, argp, sizeof(features)))
			return -EFAULT;
		return vhost_nvme_set_features(ctrl, features);
	default:
		mutex_lock(&ctrl->vdev.mutex);
		if (ioctl == VHOST_SET_OWNER) {
			r = vhost_dev_set_owner(&ctrl->vdev);
		}
		mutex_unlock(&ctrl->vdev.mutex);
		return r;
	}
}

#ifdef CONFIG_COMPAT
static long nvmet_vhost_compat_ioctl(struct file *f, unsigned int ioctl,
				     unsigned long arg)
{
	return nvmet_vhost_ioctl(f, ioctl, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations nvmet_vhost_fops = {
	.owner          = THIS_MODULE,
	.release        = nvmet_vhost_release,
	.unlocked_ioctl = nvmet_vhost_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = nvmet_vhost_compat_ioctl,
#endif
	.open           = nvmet_vhost_open,
	.llseek		= noop_llseek,
};

#if 0
static const struct nvmet_fabrics_ops nvmet_tcp_ops = {
	.owner			= THIS_MODULE,
	.type			= NVMF_TRTYPE_VHOST,
	.msdbd			= 1,
	.add_port		= nvmet_vhost_add_port,
	.remove_port		= nvmet_vhost_remove_port,
	.queue_response		= nvmet_vhost_queue_response,
	.delete_ctrl		= nvmet_vhost_delete_ctrl,
	.install_queue		= nvmet_vhost_install_queue,
	.disc_traddr		= nvmet_vhost_disc_port_addr,
};
#endif

static struct miscdevice nvmet_vhost_misc = {
	MISC_DYNAMIC_MINOR,
	"vhost-nvme",
	&nvmet_vhost_fops,
};

static int __init nvmet_vhost_init(void)
{
	return misc_register(&nvmet_vhost_misc);
}
module_init(nvmet_vhost_init);

static void nvmet_vhost_exit(void)
{
	misc_deregister(&nvmet_vhost_misc);
}
module_exit(nvmet_vhost_exit);

MODULE_AUTHOR("Ming Lin <ming.l@ssi.samsung.com>");
MODULE_LICENSE("GPL v2");
