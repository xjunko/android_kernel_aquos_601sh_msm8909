/* drivers/sharp/shtrpd/shtrpd_debug.c  (Trackpad driver)
 *
 * Copyright (C) 2015 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/slab.h>

#include <linux/types.h>
#include <linux/namei.h>
#include <linux/time.h>
#include "shtrpd.h"
#include "shtrpd_debug.h"
#include "shtrpd_debug_data.h"


/* ------------------------------------------------------------------------- */
/* DEFINE                                                                    */
/* ------------------------------------------------------------------------- */
enum {
    SHTRPD_RESULT_SUCCESS,
    SHTRPD_RESULT_FAILURE,
    SHTRPD_RESULT_FAILURE_I2C_TMO,
    NUM_SHTRPD_RESULT
};

#define SHTRPD_DBG_TRPD_ERROR_FILE               ("/durable/trackpad/trackpadlog.txt")

#define SHTRPD_DBG_ERROR_LOG_ADD_NORMAL             (0)
#define SHTRPD_DBG_ERROR_LOG_ADD_CYCLIC             (1)
#define SHTRPD_DBG_ERROR_LOG_FILE_TITLE             ("Trackpad Driver Log File\n")
#define SHTRPD_DBG_ERROR_LOG_FILE_TITLE_SIZE        (sizeof(SHTRPD_DBG_ERROR_LOG_FILE_TITLE)-1)

#define SHTRPD_DBG_ERROR_LOG_TITLE                  ("\n<Error Log>\n")
#define SHTRPD_DBG_ERROR_LOG_HEADER                 ("No,  Date,            Time,     Timezone,   Type,  Code,             Sub Code,      X,    Y,    Z,    Area, \n")
#define SHTRPD_DBG_ERROR_LOG_BLANK                  ("                                                                                                            \n")
#define SHTRPD_DBG_ERROR_LOG_END                    ("************************************************************************************************************\n")
#define SHTRPD_DBG_ERROR_LOG_NUM                    (100)

#define SHTRPD_DBG_ERROR_LOG_TITLE_SIZE             (sizeof(SHTRPD_DBG_ERROR_LOG_TITLE)-1)
#define SHTRPD_DBG_ERROR_LOG_HEADER_SIZE            (sizeof(SHTRPD_DBG_ERROR_LOG_HEADER)-1)
#define SHTRPD_DBG_ERROR_LOG_LINE_SIZE              (sizeof(SHTRPD_DBG_ERROR_LOG_BLANK)-1)
#define SHTRPD_DBG_ERROR_LOG_TOP                    (SHTRPD_DBG_ERROR_LOG_FILE_TITLE_SIZE)

#define SHTRPD_DBG_TYPE_REATI_STR                   "RE-ATI,"
#define SHTRPD_DBG_TYPE_NRST_STR                    "NRST,  "

#define SHTRPD_DBG_CODE_NONE_STR                    "----,             "
#define SHTRPD_DBG_CODE_ZILLIGAL_STR                "Z-illigal,        "
#define SHTRPD_DBG_CODE_WEAKCLINGREJECT_STR         "Weak-cling-reject,"
#define SHTRPD_DBG_CODE_INITIALSETUP_STR            "InitialSetup,     "
#define SHTRPD_DBG_CODE_INTTIMING_STR               "Int-timing,       "
#define SHTRPD_DBG_CODE_POWERKEY_STR                "PowerKey,         "
#define SHTRPD_DBG_CODE_INITIALATI_STR              "InitialATI,       "
#define SHTRPD_DBG_CODE_SHOWRESET_STR               "SHOW_RESET,       "
#define SHTRPD_DBG_CODE_FLIPOPEN_STR                "Flip-open,        "
#define SHTRPD_DBG_CODE_FLIPCLOSE_STR               "Flip-Close,       "
#define SHTRPD_DBG_CODE_PROBE_STR                   "Probe,            "

#define SHTRPD_DBG_SUBCODE_0_STR                    "0,             "

#define SHTRPD_DBG_ERROR_LOG_SUMMARY_TITLE          ("\n<Error Log Summary>\n")
#define SHTRPD_DBG_ERROR_LOG_SUMMARY_HEADER         ("Type, Code,           Sub Code,      Count \n")


#define SHTRPD_DBG_ERROR_LOG_SUMMARY_TITLE_SIZE     (sizeof(SHTRPD_DBG_ERROR_LOG_SUMMARY_TITLE)-1)
#define SHTRPD_DBG_ERROR_LOG_SUMMARY_HEADER_SIZE    (sizeof(SHTRPD_DBG_ERROR_LOG_SUMMARY_HEADER)-1)
#define SHTRPD_DBG_ERROR_LOG_SUMMARY_LINE_SIZE      (sizeof(SHTRPD_DBG_ERROR_LOG_SUMMARY_LINE01)-1)

const static const char* WeekOfDay[] = {
    "Sun"
   ,"Mon"
   ,"Tue"
   ,"Wed"
   ,"Thu"
   ,"Fri"
   ,"Sat"
};


const static const char* TypeIndex[] = {
    SHTRPD_DBG_TYPE_REATI_STR
   ,SHTRPD_DBG_TYPE_NRST_STR
};

const static const char* CodeIndex[] = {
    SHTRPD_DBG_CODE_NONE_STR
   ,SHTRPD_DBG_CODE_ZILLIGAL_STR
   ,SHTRPD_DBG_CODE_WEAKCLINGREJECT_STR
   ,SHTRPD_DBG_CODE_INITIALSETUP_STR
   ,SHTRPD_DBG_CODE_INTTIMING_STR
   ,SHTRPD_DBG_CODE_POWERKEY_STR
   ,SHTRPD_DBG_CODE_INITIALATI_STR
   ,SHTRPD_DBG_CODE_SHOWRESET_STR
   ,SHTRPD_DBG_CODE_FLIPOPEN_STR
   ,SHTRPD_DBG_CODE_FLIPCLOSE_STR
   ,SHTRPD_DBG_CODE_PROBE_STR
};

const static const char* SubCodeIndex[] = {
    SHTRPD_DBG_SUBCODE_0_STR
};


#define SHTRPD_LOG_ONE_LINE_FORMAT(length,type,code,subcode,linestring) \
    length = snprintf(bufwk, SHTRPD_DBG_ERROR_LOG_LINE_SIZE, "%03d, %04d/%02d/%02d(%s), %02d:%02d:%02d, UTC +00h,   %s%s%s%-24s",\
        ((blank_area == 1) ? i : SHTRPD_DBG_ERROR_LOG_NUM),\
        (int)(tm1.tm_year+1900), tm1.tm_mon + 1, tm1.tm_mday, WeekOfDay[tm1.tm_wday],\
        tm1.tm_hour, tm1.tm_min, tm1.tm_sec,\
        type,\
        code,\
        subcode,\
        linestring);
/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct kobject *shtrpd_debug_kobj = NULL;

struct shtrpd_dbg_reset_work_command {
    void (* proc)(void* arg, int* reset);
    void* arg;
    struct list_head list;
};

struct shtrpd_workqueue_handler_dbg_reset_log_params {
    struct shtrpd_dbg_error_code code;
    int reset;
};
extern struct shtrpd_fingers_structure shtrpd_touches_old[MAX_TOUCHES];

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* for sysfs I/F */
#if defined(CONFIG_ANDROID_ENGINEERING)
	#define SHTRPDIF_DEFINE(name, show_func, store_func) \
	static struct kobj_attribute shtrpdif_##name = \
		__ATTR(name, (S_IRUGO | S_IWUGO), show_func, store_func)
#else
	#define SHTRPDIF_DEFINE(name, show_func, store_func) \
	static struct kobj_attribute shtrpdif_##name = \
		__ATTR(name, (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP), show_func, store_func)
#endif /* SHTPS_ENGINEER_BUILD_ENABLE */


#ifdef SHTRPD_LOG_ENABLE
#if defined (CONFIG_ANDROID_ENGINEERING)
    unsigned char shtrpd_log_lv = SHTRPD_LOG_LV_ERR | SHTRPD_LOG_LV_WARN;
#else  /* CONFIG_ANDROID_ENGINEERING */
    unsigned char shtrpd_log_lv = SHTRPD_LOG_LV_ERR;
#endif /* CONFIG_ANDROID_ENGINEERING */
#endif
/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
static ssize_t shtrpd_dbg_kernel_write(struct file *fp, const char *buf, size_t size);
static ssize_t shtrpd_dbg_kernel_read(struct file *fp, char *buf, size_t size , unsigned int offset);
static ssize_t shtrpd_dbg_kernel_seek(struct file *fp, unsigned int offset);
static int shtrpd_dbg_kernel_sync(struct file *fp);
static int shtrpd_dbg_kernel_file_check_pointer(const struct file *fp, const char *buf);

static int shtrpd_dbg_add_err_log_one_line(char *buf, struct shtrpd_dbg_error_code* code);
static int shtrpd_dbg_summary_init_file(void);
static int shtrpd_dbg_add_err_log_file(struct shtrpd_dbg_error_code* code);

static int shtrpd_dbg_reset_log_output(struct shtrpd_dbg_error_code* code, int reset);
static void shtrpd_workqueue_handler_dbg_reset_log(void* arg, int* reset);
static struct shtrpd_dbg_reset_work_command* shtrpd_dbg_reset_work_alloc_command(void);
static void shtrpd_dbg_reset_work_add_command(struct shtrpd_dbg_reset_work_command* cmd);
static void shtrpd_dbg_reset_work_start(void);
static void shtrpd_dbg_reset_work_free_command(struct shtrpd_dbg_reset_work_command* cmd);
static void shtrpd_dbg_reset_work_worker(struct work_struct *wk);
static int shtrpd_dbg_api_add_err_log(struct shtrpd_dbg_error_code* code);
static DECLARE_WORK(shtrpd_dbg_reset_work_wk, shtrpd_dbg_reset_work_worker);

static DEFINE_SPINLOCK(shtrpd_dbg_reset_work_queue_lock);
static LIST_HEAD(shtrpd_dbg_reset_work_queue);
static struct workqueue_struct* shtrpd_dbg_reset_work_wq = NULL;
void shtrpd_dbg_touches_add_err_log(unsigned char type , unsigned char code , unsigned char subcode , int flag , struct shtrpd_fingers_structure* p_shtrpd_touches);
void shtrpd_dbg_init(void);

/* -----------------------------------------------------------------------------------
 */
#if 0
static ssize_t shtrpdif_show_common(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return -EINVAL;
}
#endif

static ssize_t shtrpdif_store_common(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return -EINVAL;
}

/* -----------------------------------------------------------------------------------
 */
extern int shtrpd_debug_get_fw_version(void);
static ssize_t shtrpdif_show_fwver(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ver = shtrpd_debug_get_fw_version();

	if(ver == -1){
		/* Error */
		ver = 0xFFFF;
	}
	return snprintf(buf, PAGE_SIZE, "%04X\n", ver);
}
SHTRPDIF_DEFINE(fwver, shtrpdif_show_fwver, shtrpdif_store_common);

/* -----------------------------------------------------------------------------------
 */
extern int shtrpd_debug_get_countval(unsigned char *buf, int *size);
static ssize_t shtrpdif_show_countval(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret;
	int size;
	
	ret = shtrpd_debug_get_countval((unsigned char *)buf, &size);
	if(ret < 0){
		return -1;
	}
	
	return size;
}
SHTRPDIF_DEFINE(countval, shtrpdif_show_countval, shtrpdif_store_common);

/* -----------------------------------------------------------------------------------
 */
extern int shtrpd_debug_get_ltaval(unsigned char *buf, int *size);
static ssize_t shtrpdif_show_ltaval(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret;
	int size;
	
	ret = shtrpd_debug_get_ltaval((unsigned char *)buf, &size);
	if(ret < 0){
		return -1;
	}
	
	return size;
}
SHTRPDIF_DEFINE(ltaval, shtrpdif_show_ltaval, shtrpdif_store_common);

/* -----------------------------------------------------------------------------------
 */
extern int shtrpd_debug_get_aticval(unsigned char *buf, int *size);
static ssize_t shtrpdif_show_aticval(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret;
	int size;
	
	ret = shtrpd_debug_get_aticval((unsigned char *)buf, &size);
	if(ret < 0){
		return -1;
	}
	
	return size;
}
SHTRPDIF_DEFINE(aticval, shtrpdif_show_aticval, shtrpdif_store_common);

/* -----------------------------------------------------------------------------------
 */
extern int shtrpd_debug_get_chstate(unsigned char *buf, int *size);
static ssize_t shtrpdif_show_chstate(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret;
	int size;
	
	ret = shtrpd_debug_get_chstate((unsigned char *)buf, &size);
	if(ret < 0){
		return -1;
	}
	
	return size;
}
SHTRPDIF_DEFINE(chstate, shtrpdif_show_chstate, shtrpdif_store_common);

/* -----------------------------------------------------------------------------------
 */
static struct attribute *attrs_shtrpdif[] = {
	&shtrpdif_fwver.attr,
	&shtrpdif_countval.attr,
	&shtrpdif_ltaval.attr,
	&shtrpdif_aticval.attr,
	&shtrpdif_chstate.attr,
	NULL
};

static struct attribute_group shtps_attr_grp_shtrpdif = {
	.name = "shtrpdif",
	.attrs = attrs_shtrpdif,
};

/*  -----------------------------------------------------------------------------------
 */
struct kobject* shtrpd_init_debug_sysfs(void)
{
	shtrpd_debug_kobj = kobject_create_and_add("shtrpd", kernel_kobj);
	if(shtrpd_debug_kobj == NULL){
		pr_err("kobj create failed : shtrpd\n");
	}else{
		if(sysfs_create_group(shtrpd_debug_kobj, &shtps_attr_grp_shtrpdif)){
			pr_err("kobj create failed : shtps_attr_grp_shtrpdif\n");
		}
	}
	return shtrpd_debug_kobj;
}
EXPORT_SYMBOL(shtrpd_init_debug_sysfs);

void shtrpd_deinit_debug_sysfs(void)
{
	if(shtrpd_debug_kobj != NULL){
		sysfs_remove_group(shtrpd_debug_kobj, &shtps_attr_grp_shtrpdif);
		kobject_put(shtrpd_debug_kobj);
	}
}
EXPORT_SYMBOL(shtrpd_deinit_debug_sysfs);

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_kernel_write                                                   */
/* ------------------------------------------------------------------------- */
static ssize_t shtrpd_dbg_kernel_write(struct file *fp, const char *buf, size_t size)
{
    mm_segment_t old_fs;
    ssize_t res = 1;

    if(! shtrpd_dbg_kernel_file_check_pointer(fp, buf)){
        return 0;
    }

    old_fs = get_fs();
    set_fs(get_ds());
    res = fp->f_op->write(fp, buf, size, &fp->f_pos);
    set_fs(old_fs);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_kernel_read                                                    */
/* ------------------------------------------------------------------------- */
static ssize_t shtrpd_dbg_kernel_read(struct file *fp, char *buf, size_t size , unsigned int offset)
{
    mm_segment_t old_fs;
    ssize_t res = 0;
    loff_t fpos = offset;

    if(! shtrpd_dbg_kernel_file_check_pointer(fp, buf)){
        return 0;
    }

    old_fs = get_fs();
    set_fs(get_ds());
    res = fp->f_op->read(fp, buf, size, &fpos);
    set_fs(old_fs);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_kernel_seek                                                    */
/* ------------------------------------------------------------------------- */
static ssize_t shtrpd_dbg_kernel_seek(struct file *fp, unsigned int offset)
{
    ssize_t res;
    loff_t fpos;

    if(! shtrpd_dbg_kernel_file_check_pointer(fp, SHTRPD_DBG_KERNEL_FILE_CHECK_POINTER_ALWAYS_OK)){
        return 0;
    }

    fpos = offset;
    res = fp->f_op->llseek(fp, fpos, SEEK_SET);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_kernel_sync                                                    */
/* ------------------------------------------------------------------------- */
static int shtrpd_dbg_kernel_sync(struct file *fp)
{
    int res;

    if(! shtrpd_dbg_kernel_file_check_pointer(fp, SHTRPD_DBG_KERNEL_FILE_CHECK_POINTER_ALWAYS_OK)){
        return 0;
    }

    res = fp->f_op->fsync(fp, 0, LLONG_MAX, 0);

    return res;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_kernel_file_check_pointer                                      */
/* ------------------------------------------------------------------------- */
static int shtrpd_dbg_kernel_file_check_pointer(const struct file *fp, const char *buf)
{
    int res = 1;

    if( IS_ERR_OR_NULL(fp) ){
        SHTRPD_ERROR("<fp_INVALID_POINTER>\n");
        res = 0;
    }

    if( buf == NULL ){
        SHTRPD_ERROR("<buf_NULL_POINTER>\n");
        res = 0;
    }
    
    return res;
}

/*---------------------------------------------------------------------------*/
/*      shtrpd_dbg_touches_add_err_log                                       */
/*---------------------------------------------------------------------------*/
void shtrpd_dbg_touches_add_err_log(unsigned char type , unsigned char code , unsigned char subcode , int flag , struct shtrpd_fingers_structure* p_shtrpd_touches)
{
    char linestring_wk[25] = {};
#if 0
    int i;
#endif
    struct shtrpd_dbg_error_code errorcode;

    errorcode.type = type;
    errorcode.code = code;
    errorcode.subcode = subcode;

    if ( p_shtrpd_touches == NULL ){
        p_shtrpd_touches = shtrpd_touches_old;
    }

    memset( &errorcode.linestring[0], 0, 24 );

    if ( flag < 0 ){
        flag = - 1;
    }
#if 0
    for (i = 0 ; i < MAX_TOUCHES ; i ++){
#endif
        if(flag & 1){
            snprintf(&linestring_wk[0],  7, "%d,    ", p_shtrpd_touches->XPos);
            snprintf(&linestring_wk[6],  7, "%d,    ", p_shtrpd_touches->YPos);
            snprintf(&linestring_wk[12], 7, "%d,    ", p_shtrpd_touches->touchStrength);
            sprintf(&linestring_wk[18],"%d,", p_shtrpd_touches->area);
			memcpy(&errorcode.linestring[0],&linestring_wk[0],24 );
            shtrpd_dbg_reset_log_output(&errorcode,0);
        }
        flag >>= 1;
        p_shtrpd_touches ++;
#if 0
    }
#endif
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_init                                                           */
/* ------------------------------------------------------------------------- */
void shtrpd_dbg_init(void)
{
    shtrpd_dbg_reset_work_wq = create_singlethread_workqueue("shtrpd_dbg_reset_work_wq");
    if (!shtrpd_dbg_reset_work_wq) {
        SHTRPD_ERROR("shtrpd_dbg_reset_work_wq create failed.\n" );
    }
}


/*---------------------------------------------------------------------------*/
/*      shtrpd_dbg_reset_log_output                                          */
/*---------------------------------------------------------------------------*/
static int shtrpd_dbg_reset_log_output(struct shtrpd_dbg_error_code* code, int reset)
{
    struct shtrpd_dbg_reset_work_command* cmd = NULL;
    struct shtrpd_workqueue_handler_dbg_reset_log_params* params = NULL;
    int error = -EINVAL;

    SHTRPD_TRACE("in\n");
    params = (struct shtrpd_workqueue_handler_dbg_reset_log_params*)kzalloc((sizeof(struct shtrpd_workqueue_handler_dbg_reset_log_params)+30), GFP_KERNEL);
    if (params == NULL) {
        SHTRPD_ERROR("allocate parameter error. no memory\n");
        error = -ENOMEM;
        goto errout;
    }

    params->code = *code;
    params->reset = reset;

    cmd = shtrpd_dbg_reset_work_alloc_command();
    if (cmd == NULL) {
        SHTRPD_ERROR("allocate command error. no memory\n");
        error = -ENOMEM;
        goto errout;
    }

    cmd->proc = shtrpd_workqueue_handler_dbg_reset_log;
    cmd->arg = params;

    shtrpd_dbg_reset_work_add_command(cmd);
    shtrpd_dbg_reset_work_start();

    SHTRPD_TRACE("out normaly finished.\n");
    return 0;

errout:
    if (cmd != NULL) {
        shtrpd_dbg_reset_work_free_command(cmd);
    }
    if (params != NULL) {
        kfree(params);
    }

    SHTRPD_ERROR("abnormaly finished. error=%d\n", error);
    return error;
}

/*---------------------------------------------------------------------------*/
/*      shtrpd_workqueue_handler_dbg_reset_log                               */
/*---------------------------------------------------------------------------*/
static void shtrpd_workqueue_handler_dbg_reset_log(void* arg, int* reset)
{
    struct shtrpd_workqueue_handler_dbg_reset_log_params* params;

    SHTRPD_TRACE("in\n");
    
    params = (struct shtrpd_workqueue_handler_dbg_reset_log_params*)arg;

    shtrpd_dbg_api_add_err_log(&params->code);

    if (params->reset == 1) {
        *reset = 1;
    }

    kfree(params);

    SHTRPD_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */		
/* shtrpd_dbg_reset_work_alloc_command                                       */		
/* ------------------------------------------------------------------------- */		
struct shtrpd_dbg_reset_work_command* shtrpd_dbg_reset_work_alloc_command(void)		
{		
    struct shtrpd_dbg_reset_work_command* cmd;		
    cmd = (struct shtrpd_dbg_reset_work_command*)kzalloc(sizeof(struct shtrpd_dbg_reset_work_command), GFP_KERNEL);		
   if (cmd == NULL) {		
        SHTRPD_ERROR("kzalloc() failure.\n");		
    }		
    return cmd;		
}		
		
/* ------------------------------------------------------------------------- */		
/* shtrpd_dbg_reset_work_free_command                                        */		
/* ------------------------------------------------------------------------- */		
static void shtrpd_dbg_reset_work_free_command(struct shtrpd_dbg_reset_work_command* cmd)		
{		
    if (cmd != NULL) {		
        kfree(cmd);		
    } else {		
        SHTRPD_ERROR("null pointer.\n");		
    }		
}		

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_reset_work_add_command                                         */
/* ------------------------------------------------------------------------- */
static void shtrpd_dbg_reset_work_add_command(struct shtrpd_dbg_reset_work_command* cmd)
{
    SHTRPD_DEBUG("add queue: proc=%pS arg=%p\n", cmd->proc, cmd->arg);

    spin_lock(&shtrpd_dbg_reset_work_queue_lock);
    list_add_tail(&cmd->list, &shtrpd_dbg_reset_work_queue);
    spin_unlock(&shtrpd_dbg_reset_work_queue_lock);
}


/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_reset_work_start                                               */
/* ------------------------------------------------------------------------- */
static void shtrpd_dbg_reset_work_start(void)
{
    SHTRPD_TRACE("in\n");

    if (shtrpd_dbg_reset_work_wq != NULL) {
        if (queue_work(shtrpd_dbg_reset_work_wq, &shtrpd_dbg_reset_work_wk) == 0) {
            SHTRPD_DEBUG("work already pending.\n");
        }
    } else {
        SHTRPD_ERROR("workqueue not created.\n");
    }
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_reset_work_worker                                              */
/* ------------------------------------------------------------------------- */
static void shtrpd_dbg_reset_work_worker(struct work_struct *wk)
{
    struct list_head* item;
    struct shtrpd_dbg_reset_work_command* cmd;
    int reset = 0;

    SHTRPD_TRACE("in\n");

    for (;;) {
        item = NULL;

        spin_lock(&shtrpd_dbg_reset_work_queue_lock);
        if (!list_empty(&shtrpd_dbg_reset_work_queue)) {
            item = shtrpd_dbg_reset_work_queue.next;
            list_del(item);
        }
        spin_unlock(&shtrpd_dbg_reset_work_queue_lock);

        if (item == NULL) {
            break;
        }

        cmd = list_entry(item, struct shtrpd_dbg_reset_work_command, list);
        SHTRPD_DEBUG("execute: proc=%pS arg=%p\n", cmd->proc, cmd->arg);
        cmd->proc(cmd->arg, &reset);

        shtrpd_dbg_reset_work_free_command(cmd);
    }

    SHTRPD_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_api_add_err_log                                                */
/* ------------------------------------------------------------------------- */
int shtrpd_dbg_api_add_err_log(struct shtrpd_dbg_error_code* code)
{
    int ret;

    SHTRPD_TRACE("in\n");

    if (code->type    >= SHTRPD_DBG_TYPE_MAX ||
        code->code    >= SHTRPD_DBG_CODE_MAX ||
        code->subcode >= SHTRPD_DBG_SUBCODE_MAX){
        SHTRPD_ERROR("parameter error.\n");
        return SHTRPD_RESULT_FAILURE;
    }
    ret = shtrpd_dbg_add_err_log_file(code);

    SHTRPD_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_add_err_log_file                                               */
/* ------------------------------------------------------------------------- */
static int shtrpd_dbg_add_err_log_file(struct shtrpd_dbg_error_code *code)
{
    struct path  path;
    struct file *fp;
    unsigned int offset;
    char *buf;
    int ret = -EINVAL;
    size_t size = (SHTRPD_DBG_ERROR_LOG_LINE_SIZE * (SHTRPD_DBG_ERROR_LOG_NUM + 1));

    SHTRPD_TRACE("in\n");
    ret = kern_path(SHTRPD_DBG_TRPD_ERROR_FILE, LOOKUP_OPEN, &path);
    if (ret != 0) {
        ret = shtrpd_dbg_summary_init_file();
        if (ret != SHTRPD_RESULT_SUCCESS) {
            return SHTRPD_RESULT_FAILURE;
        }
    }
    offset =  SHTRPD_DBG_ERROR_LOG_TOP;
    offset += SHTRPD_DBG_ERROR_LOG_TITLE_SIZE;
    offset += SHTRPD_DBG_ERROR_LOG_HEADER_SIZE;

    buf = kzalloc((size + SHTRPD_DBG_ERROR_LOG_LINE_SIZE * 2), GFP_KERNEL);
    if (!buf) {
        SHTRPD_ERROR("allocate read buffer error. [no memory]\n");
        return -ENOMEM;
    }
    fp = filp_open(SHTRPD_DBG_TRPD_ERROR_FILE, O_RDWR, 0660);
    if (IS_ERR_OR_NULL(fp)) {
        kfree(buf);
        SHTRPD_ERROR("Cannot open file: %s err=%p\n", SHTRPD_DBG_TRPD_ERROR_FILE, fp);
        return SHTRPD_RESULT_FAILURE;
    }
    shtrpd_dbg_kernel_read(fp, buf, size, offset);

    ret = shtrpd_dbg_add_err_log_one_line(buf, code);
    shtrpd_dbg_kernel_seek(fp, offset);

    if (ret == SHTRPD_DBG_ERROR_LOG_ADD_NORMAL) {
        shtrpd_dbg_kernel_write(fp, buf, size);
    } else {
        shtrpd_dbg_kernel_write(fp, buf + SHTRPD_DBG_ERROR_LOG_LINE_SIZE, size);
    }
    kfree(buf);
    {
        int res;
        if ((res = shtrpd_dbg_kernel_sync(fp)) != 0) {
            SHTRPD_ERROR("fsync result: %d\n", res);
        }
    }
    filp_close(fp, NULL);

    SHTRPD_TRACE("out\n");
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_add_err_log_one_line                                           */
/* ------------------------------------------------------------------------- */
static int shtrpd_dbg_add_err_log_one_line(char *buf, struct shtrpd_dbg_error_code* code)
{
    struct timeval tv;
    struct tm tm1, tm2;
    int i, blank_area = 0, length = 0;
    char *bufwk;

    bufwk = buf;

    do_gettimeofday(&tv);
    time_to_tm((time_t)tv.tv_sec, 0, &tm1);
    time_to_tm((time_t)tv.tv_sec, (sys_tz.tz_minuteswest*60*(-1)), &tm2);

    for (i = 1; i <= SHTRPD_DBG_ERROR_LOG_NUM; i++) {
        if (*bufwk == '*') {
            blank_area = 1;
            break;
        }
        else {
            bufwk += SHTRPD_DBG_ERROR_LOG_LINE_SIZE;
        }
    }
    SHTRPD_LOG_ONE_LINE_FORMAT(length,TypeIndex[code->type],CodeIndex[code->code],SubCodeIndex[code->subcode],code->linestring)

    bufwk[length]='\n';

    memcpy(bufwk + SHTRPD_DBG_ERROR_LOG_LINE_SIZE, SHTRPD_DBG_ERROR_LOG_END, SHTRPD_DBG_ERROR_LOG_LINE_SIZE);

    if (!blank_area) {
        bufwk = buf + SHTRPD_DBG_ERROR_LOG_LINE_SIZE;
        for (i = 1; i < SHTRPD_DBG_ERROR_LOG_NUM; i++) {
            *(bufwk+0) = '0'+(i/100);
            *(bufwk+1) = '0'+((i%100)/10);
            *(bufwk+2) = '0'+(i%10);
            bufwk += SHTRPD_DBG_ERROR_LOG_LINE_SIZE;
        }
        return SHTRPD_DBG_ERROR_LOG_ADD_CYCLIC;
    }
    return SHTRPD_DBG_ERROR_LOG_ADD_NORMAL;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_dbg_summary_init_file                                              */
/* ------------------------------------------------------------------------- */
static int shtrpd_dbg_summary_init_file(void)
{
    struct file *fp;
    int i;

    SHTRPD_TRACE("in\n");

    SHTRPD_DEBUG("open file: %s pid=%d tgid=%d comm=%s\n", SHTRPD_DBG_TRPD_ERROR_FILE, current->pid, current->tgid,
                                                                                                       current->comm);
    fp = filp_open(SHTRPD_DBG_TRPD_ERROR_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0660);
    if (IS_ERR_OR_NULL(fp)) {
        SHTRPD_ERROR("Cannot create file: %s err=%d pid=%d tgid=%d comm=%s\n", SHTRPD_DBG_TRPD_ERROR_FILE, (int)fp, current->pid, current->tgid, current->comm);
        return SHTRPD_RESULT_FAILURE;
    }

    shtrpd_dbg_kernel_write(fp, SHTRPD_DBG_ERROR_LOG_FILE_TITLE, SHTRPD_DBG_ERROR_LOG_FILE_TITLE_SIZE);
    shtrpd_dbg_kernel_write(fp, SHTRPD_DBG_ERROR_LOG_TITLE, SHTRPD_DBG_ERROR_LOG_TITLE_SIZE);
    shtrpd_dbg_kernel_write(fp, SHTRPD_DBG_ERROR_LOG_HEADER, SHTRPD_DBG_ERROR_LOG_HEADER_SIZE);

    shtrpd_dbg_kernel_write(fp, SHTRPD_DBG_ERROR_LOG_END, SHTRPD_DBG_ERROR_LOG_LINE_SIZE);
    for (i = 1; i < SHTRPD_DBG_ERROR_LOG_NUM + 1; i++) {
        shtrpd_dbg_kernel_write(fp, SHTRPD_DBG_ERROR_LOG_BLANK, SHTRPD_DBG_ERROR_LOG_LINE_SIZE);
    }

    filp_close(fp, NULL);

    SHTRPD_TRACE("out\n");
    return SHTRPD_RESULT_SUCCESS;
}

