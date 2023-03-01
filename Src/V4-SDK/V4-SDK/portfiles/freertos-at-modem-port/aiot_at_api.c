/**
 * @file aiot_at_api.c
 * @brief 提供了外部通信模组对接到SDK网络适配层的接口实现
 * @date 2020-01-20
 *
 * @copyright Copyright (C) 2015-2020 Alibaba Group Holding Limited
 *
 * @details
 * 1. 本实现基于合宙Air724 LTE模组, 用户可简单修改AT指令集适配到其他模组上
 * 2. 本实现提供了基于TCP AT通信指令的网络数据收发能力, 用户可参考代码修改到UDP或TLS等其他通信协议
 * 3. 支持多条数据链路同时收发的情况
 * 4. 用户应根据应用的实际数据吞吐量合理配置ringbuf大小, ringbuf写入溢出会导致报文不完整, 设备会重新建连
 */

#include "core_stdinc.h"
#include "core_string.h"
#include "aiot_state_api.h"
#include "aiot_sysdep_api.h"
#include "aiot_at_api.h"
#include "core_log.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>


#define AT_RSP_LEN_MINIMUM			(3)
#define AT_RSP_TCP_RECV_HEAD 		"+RECEIVE,"
#define AT_CMD_TCP_SETUP_FMT		"AT+CIPSTART=%d,TCP,%s,%d\r\n"
#define AT_CMD_TCP_SEND_FMT			"AT+CIPSEND=%d,%d\r\n"



static int32_t core_at_uart_tx(const uint8_t *p_data, uint16_t len, uint32_t timeout);

/* 模块初始化命令列表 */
static const core_at_cmd_item_t at_bootstrap_cmd_list[] = {
	{	/* UART通道测试 */
		.cmd = "AT\r\n",
		.cmd_len = sizeof("AT\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 关闭回显 */
		.cmd = "ATE0\r\n",
		.cmd_len = sizeof("ATE0\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 获取通信模块 IMEI 号 */
		.cmd = "AT+CGSN\r\n",
		.cmd_len = sizeof("AT+CGSN\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 获取模组固件版本号 */
		.cmd = "AT+CGMR\r\n",
		.cmd_len = sizeof("AT+CGMR\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 检查SIM卡 */
		.cmd = "AT+CPIN?\r\n",
		.cmd_len = sizeof("AT+CPIN?\r\n") - 1,
		.rsp = "READY",
		.handler = NULL,
	},
	{	/* 获取SIM卡IMSI */
		.cmd = "AT+CIMI\r\n",
		.cmd_len = sizeof("AT+CIMI\r\n") - 1,
		.rsp = "READY",
		.handler = NULL,
	},
	{	/* 检查信号强度 */
		.cmd = "AT+CSQ\r\n",
		.cmd_len = sizeof("AT+CSQ\r\n") - 1,
		.rsp = "READY",
		.handler = NULL,
	},
	{	/* 手动网络附着 */
		.cmd = "AT+CGATT=1\r\n",
		.cmd_len = sizeof("AT+CGATT=1\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 设置APN */
		.cmd = "AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n",
		.cmd_len = sizeof("AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 先关闭TCP连接 */
		.cmd = "AT+CIPSHUT\r\n",
		.cmd_len = sizeof("AT+CIPSHUT\r\n") - 1,
		.rsp = "SHUT OK",
		.handler = NULL,
	},
	{	/* 打开TCP多连接 */
		.cmd = "AT+CIPMUX=1\r\n",
		.cmd_len = sizeof("AT+CIPMUX=1\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 设置快速发送 */
		.cmd = "AT+CIPQSEND=1\r\n",
		.cmd_len = sizeof("AT+CIPQSEND=1\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 启动任务 */
		.cmd = "AT+CSTT\r\n",
		.cmd_len = sizeof("AT+CSTT\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 激活移动场景,发起GPRS/CSD无线连接 */
		.cmd = "AT+CIICR\r\n",
		.cmd_len = sizeof("AT+CIICR\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
	{	/* 查询本地IP地址 */
		.cmd = "AT+CIFSR\r\n",
		.cmd_len = sizeof("AT+CIFSR\r\n") - 1,
		.rsp = ".",
		.handler = NULL,
	},
	{	/* 2 发送报文不显示'>', "SEND OK"/"DATA ACCEPT" */
		.cmd = "AT+CIPSPRT=2\r\n",
		.cmd_len = sizeof("AT+CIPSPRT=2\r\n") - 1,
		.rsp = "OK",
		.handler = NULL,
	},
};

/* TCP建立连接AT命令列表 */
static core_at_cmd_item_t at_connect_cmd_list[] = {
	{	/* 建立TCP连接, TODO: aiot_at_nwk_connect接口会组织此AT命令 */
		.cmd = NULL,	/* "AT+CIPSTART=0,%s,%d\r\n" */
		.rsp = "OK&CONNECT OK", /*有两条回应报文都要校验*/
		.handler = NULL,
	},
};

/* TCP关闭连接AT命令列表 */
static const core_at_cmd_item_t at_disconn_cmd_list[] = {
	{	/* 建立TCP连接 */
		.cmd = "AT+CIPSHUT\r\n",
		.cmd_len = sizeof("AT+CIPSHUT\r\n") - 1,
		.rsp = "SHUT OK",
		.handler = NULL,
	}
};

/* external variables */
extern aiot_sysdep_portfile_t g_aiot_sysdep_portfile;

/* local variables */
static aiot_sysdep_portfile_t *at_sysdep = &g_aiot_sysdep_portfile;


static core_at_handle_t at_handle = {
	.is_init = 0,
};

/*** ringbuf start ***/
int32_t core_ringbuf_init(core_ringbuf_t *rbuf, uint32_t size)
{
    if (rbuf->buf) {
        return STATE_AT_ALREADY_INITED;
    }
	memset(rbuf, 0, sizeof(core_ringbuf_t));

    rbuf->buf = at_sysdep->core_sysdep_malloc(size, MODULE_NAME_AT);
    if (NULL == rbuf->buf) {
        return STATE_SYS_DEPEND_MALLOC_FAILED;
    }

    rbuf->head = rbuf->buf;
    rbuf->tail = rbuf->buf;
    rbuf->end = rbuf->buf + size;
    rbuf->size = size;
    rbuf->mutex = at_sysdep->core_sysdep_mutex_init();

    return STATE_SUCCESS;
}
void core_ringbuf_reset(core_ringbuf_t *rbuf)
{
	AIOT_UART_DISABLE_RX_IT();
	rbuf->tail = rbuf->buf;
	rbuf->head = rbuf->buf;
	AIOT_UART_ENABLE_RX_IT();
}

int32_t core_ringbuf_get_occupy(core_ringbuf_t *rbuf)
{
	uint32_t used = 0;

	AIOT_UART_DISABLE_RX_IT();
    if (rbuf->tail >= rbuf->head) {
        used = rbuf->tail - rbuf->head;
    }
    else {
        used = rbuf->tail - rbuf->buf + rbuf->end - rbuf->head;
    }
	AIOT_UART_ENABLE_RX_IT();

	return used;
}

int32_t core_ringbuf_write(core_ringbuf_t *rbuf, const uint8_t *data, uint32_t len)
{
    if (len > (rbuf->size - core_ringbuf_get_occupy(rbuf))) {
    	return STATE_AT_RINGBUF_OVERFLOW;
    }

    if (rbuf->tail + len >= rbuf->end) {
        uint32_t remain_len = rbuf->end - rbuf->tail;

        memcpy(rbuf->tail, data, remain_len);
        memcpy(rbuf->buf, data + remain_len, len - remain_len);
        rbuf->tail = rbuf->buf + len - remain_len;
    }
    else {
        memcpy(rbuf->tail, data, len);
        rbuf->tail += len;
    }

    return len;
}

// todo ringbuf mutex
int32_t core_ringbuf_read(core_ringbuf_t *rbuf, uint8_t *data, uint32_t len)
{
	int32_t used = core_ringbuf_get_occupy(rbuf);

    if (len > used) {
        return 0;
    }

    if (rbuf->head + len >= rbuf->end) {
        uint32_t remain_len = rbuf->end - rbuf->head;

        memcpy(data, rbuf->head, remain_len);
        memcpy(data + remain_len, rbuf->buf, len - remain_len);
        rbuf->head = rbuf->buf + len - remain_len;
    }
    else {
        memcpy(data, rbuf->head, len);
        rbuf->head += len;
    }

    return len;
}

void core_ringbuf_deinit(core_ringbuf_t *rbuf)
{
    if (NULL == rbuf) {
        return;
    }

    if (rbuf->buf) {
        at_sysdep->core_sysdep_free(rbuf->buf);
    }

    memset(rbuf, 0, sizeof(core_ringbuf_t));
}
/*** ringbuf end ***/

int32_t core_ringbuf_read_sync_rsp(uint8_t *rsp, uint32_t len, uint32_t timeout)
{
	uint64_t timeout_start = at_sysdep->core_sysdep_time();
	uint32_t rsp_len = 0;
	uint32_t temp = 0;

	/* try to read out all data in ringbuf */
	do {
		at_sysdep->core_sysdep_sleep(AIOT_AT_RINGBUF_RETRY_INTERVAL);
		rsp_len = core_ringbuf_get_occupy(&at_handle.rsp_rb);
		if (rsp_len == temp && rsp_len != 0) {
			break;
		}
		temp = rsp_len;
	} while ((at_sysdep->core_sysdep_time() - timeout_start) < timeout);

	return core_ringbuf_read(&at_handle.rsp_rb, rsp, rsp_len);
}

int32_t core_at_commands_send_sync(core_at_handle_t *handle, const core_at_cmd_item_t *cmd_list, uint16_t cmd_num)
{
	uint16_t i = 0;
	uint16_t retry_cnt = AIOT_AT_CMD_RETRY_TIME;
	int32_t res = STATE_SUCCESS;

	if (NULL == cmd_list || 0 == cmd_num) {
		return STATE_USER_INPUT_NULL_POINTER;
	}
	core_ringbuf_reset(&at_handle.rsp_rb);
	for (; i < cmd_num; i++)
	{
		char* rsp_start = cmd_list[i].rsp;
		char* rep_end = cmd_list[i].rsp + strlen(cmd_list[i].rsp);
		char* result = rsp_start;
		char rsp[AIOT_AT_RSP_LEN_MAXIMUM] = {0};
		char tmp[AIOT_AT_RSP_LEN_MAXIMUM] = {0};

		/* 发送AT命令 */
		//core_log_hexdump(-1, 1, (unsigned char *)cmd_list[i].cmd, cmd_list[i].cmd_len);
		printf("[%llu][cmd] %s\r\n", at_sysdep->core_sysdep_time(), (uint8_t *)cmd_list[i].cmd);
		res = core_at_uart_tx((uint8_t *)cmd_list[i].cmd, cmd_list[i].cmd_len, handle->tx_timeout);
		if (res != cmd_list[i].cmd_len) {
			res = STATE_AT_UART_TX_FAILED;
			break;
		}

		memset(tmp, 0, sizeof(tmp));

		while(result && rsp_start && handle->rsp_timeout > 0 && strlen(rsp_start) > 0)
		{
			/* 获取应答 */
			res = core_ringbuf_read_sync_rsp((uint8_t *)rsp, sizeof(rsp), handle->rsp_timeout);
			if (res < 0) {
				if (--retry_cnt > 0) {
					i--;
					continue;
				} else {
					break;
				}
			}
			printf("[%llu][rsp] %s\r\n", at_sysdep->core_sysdep_time(), (uint8_t *)rsp);
			if((result = strchr(rsp_start, '&')) != NULL)
			{
				rep_end = result;
			}
			memcpy(tmp, rsp_start, rep_end - rsp_start);
			if (strstr(rsp, tmp) == NULL) {
				res = STATE_AT_GET_RSP_FAILED;
				break;
			}
			rsp_start = rep_end + 1;
			rep_end = cmd_list[i].rsp + strlen(cmd_list[i].rsp);
		}
		

		/* call user handle */
		if (cmd_list[i].handler) {
			if ((res = cmd_list[i].handler(rsp, handle->user_data)) < STATE_SUCCESS) {
				continue;
			}
		}

		retry_cnt = AIOT_AT_CMD_RETRY_TIME;
		res = STATE_SUCCESS;
	}

	return res;
}

static int32_t core_at_uart_tx(const uint8_t *p_data, uint16_t len, uint32_t timeout)
{
	int32_t res = STATE_SUCCESS;

	if (at_handle.uart_tx_func == NULL) {
		return STATE_AT_UART_TX_FUNC_NULL;
	}

	at_sysdep->core_sysdep_mutex_lock(at_handle.tx_mutex);
	res = at_handle.uart_tx_func(p_data, len, timeout);
	at_sysdep->core_sysdep_mutex_unlock(at_handle.tx_mutex);
	return res;
}

/**
 * AT moduel API start
 */
int32_t aiot_at_init(void)
{
	int32_t res = STATE_SUCCESS;

	if (at_handle.is_init != 0) {
		return STATE_AT_ALREADY_INITED;
	}

	memset(&at_handle, 0, sizeof(core_at_handle_t));
	at_handle.tx_timeout = AIOT_AT_TX_TIMEOUT_DEFAULT;
	at_handle.rsp_timeout = AIOT_AT_RX_TIMEOUT_DEFAULT;

	at_handle.tx_mutex = at_sysdep->core_sysdep_mutex_init();

	if ((res = core_ringbuf_init(&at_handle.rsp_rb, AIOT_AT_DATA_RB_SIZE_DEFAULT)) < STATE_SUCCESS) {
		return res;
	}

	at_handle.is_init = 1;
	return res;
}

int32_t aiot_at_setopt(aiot_at_option_t opt, void *data)
{
	if (at_handle.is_init != 1) {
		return STATE_AT_NOT_INITED;
	}
	if (NULL == data ) {
		return STATE_USER_INPUT_NULL_POINTER;
	}
	if (opt >= AIOT_ATOPT_MAX) {
		return STATE_USER_INPUT_OUT_RANGE;
	}

	switch (opt) {
		case AIOT_ATOPT_UART_TX_FUNC: {
			at_handle.uart_tx_func = (aiot_at_uart_tx_func_t)data;
		} break;
		case AIOT_ATOPT_USER_DATA: {
			at_handle.user_data = data;
		} break;
		default:
			break;
	}

	return STATE_SUCCESS;
}

int32_t aiot_at_bootstrap(void)
{
	if (at_handle.is_init != 1) {
		return STATE_AT_NOT_INITED;
	}

	return core_at_commands_send_sync(&at_handle,
								      at_bootstrap_cmd_list,
									  sizeof(at_bootstrap_cmd_list)/sizeof(at_bootstrap_cmd_list[0]));
}

int32_t aiot_at_nwk_open(uint8_t *socket_id)
{
	uint8_t i = 0;
	int32_t res = STATE_SUCCESS;

	if (at_handle.is_init != 1) {
		return STATE_AT_NOT_INITED;
	}

	for (i = 0; i < sizeof(at_handle.fd) / sizeof(at_handle.fd[0]); i++) {
		if (at_handle.fd[i].link_status == CORE_AT_LINK_DISABLE) {
			if ((res = core_ringbuf_init(&at_handle.fd[i].data_rb, AIOT_AT_RSP_RB_SIZE_DEFAULT)) < STATE_SUCCESS) {
				return res;
			}

			*socket_id = i;
			at_handle.fd[i].link_status = CORE_AT_LINK_DISCONN;
			return STATE_SUCCESS;
		}
	}

	return STATE_AT_NO_AVAILABLE_LINK;
}

int32_t aiot_at_nwk_connect(uint8_t socket_id, const char *host, uint16_t port, uint32_t timeout)
{
	(void)timeout;		/* 超时时间不由外部指定, 使用AT组件配置的时间 */
	char conn_cmd[AIOT_AT_CMD_LEN_MAXIMUM] = {0};
	int32_t res = STATE_SUCCESS;

	if (at_handle.is_init != 1) {
		return STATE_AT_NOT_INITED;
	}
	if (socket_id >= sizeof(at_handle.fd) / sizeof(at_handle.fd[0])) {
		return STATE_USER_INPUT_OUT_RANGE;
	}

	/* 拼装host, port */
	memset(conn_cmd, 0, sizeof(conn_cmd));
	snprintf(conn_cmd, sizeof(conn_cmd), AT_CMD_TCP_SETUP_FMT, socket_id, host, port);
	at_connect_cmd_list[0].cmd = conn_cmd;
	at_connect_cmd_list[0].cmd_len = strlen(conn_cmd) - 1;
	/* send tcp setup command */
	res = core_at_commands_send_sync(&at_handle, at_connect_cmd_list,
									 sizeof(at_connect_cmd_list) / sizeof(core_at_cmd_item_t));
	if (res == STATE_SUCCESS) {
		at_handle.fd[socket_id].link_status = CORE_AT_LINK_CONN;
	}

	return res;
}

int32_t _at_rsp_delay_handler(char *rsp, void *user_data)
{
	at_sysdep->core_sysdep_sleep(200);
	return 0;
}

int32_t aiot_at_nwk_send(uint8_t socket_id, const uint8_t *buffer, uint32_t len, uint32_t timeout)
{
	(void)timeout;			/* 发送超时有at组件内部参数决定 */
	char cmd_buf[AIOT_AT_CMD_LEN_MAXIMUM] = {0};
	char rev_buf[AIOT_AT_CMD_LEN_MAXIMUM] = {0};
	int32_t res = STATE_SUCCESS;
	if (at_handle.is_init != 1) {
		return STATE_AT_NOT_INITED;
	}
	if (NULL == buffer) {
		return STATE_USER_INPUT_NULL_POINTER;
	}
	if (0 == len) {
		return STATE_USER_INPUT_OUT_RANGE;
	}

	if (socket_id >= sizeof(at_handle.fd) / sizeof(at_handle.fd[0])) {
		return STATE_USER_INPUT_OUT_RANGE;
	}
	if (at_handle.fd[socket_id].link_status <= CORE_AT_LINK_DISCONN) {
		return STATE_AT_LINK_IS_DISCONN;
	}

	/* 组装'TCP数据发送'AT命令 */
	snprintf(cmd_buf, sizeof(cmd_buf), AT_CMD_TCP_SEND_FMT, socket_id, (int)len);
	snprintf(rev_buf, sizeof(rev_buf), "DATA ACCEPT:%d,%d", socket_id, (int)len);
	core_at_cmd_item_t tcp_send_cmd_list[] = {
		{
			.cmd = cmd_buf,
			.cmd_len = strlen(cmd_buf),
			.rsp = ">",
			.handler = NULL,	/* 收到`>`字符后做短延时处理, N720模组要求 */
		},
		{
			.cmd = (char *)buffer,
			.cmd_len = len,
			.rsp = rev_buf,
			.handler = NULL,
		},
	};

	/*发送完成数据不等待回应,等待回应会出现10ms左右延时,影响效率*/
	uint64_t timebak = at_handle.rsp_timeout;
	at_handle.rsp_timeout = 0;
	res = core_at_commands_send_sync(&at_handle, tcp_send_cmd_list,
							sizeof(tcp_send_cmd_list) / sizeof(core_at_cmd_item_t));
	at_handle.rsp_timeout = timebak;
	if (res >= STATE_SUCCESS) {
		//core_log(at_sysdep, 100, "aiot_at_nwk_send success\r\n");
		return len;
	}
	return res;
}

int32_t aiot_at_nwk_recv(uint8_t socket_id, uint8_t *buffer, uint32_t len, uint32_t timeout_ms)
{
	uint64_t timeout_start = at_sysdep->core_sysdep_time();
	int32_t res = STATE_SUCCESS;
	if (at_handle.is_init != 1) {
		return STATE_AT_NOT_INITED;
	}

	if (socket_id >= sizeof(at_handle.fd) / sizeof(at_handle.fd[0])) {
		return STATE_USER_INPUT_OUT_RANGE;
	}

	if (at_handle.fd[socket_id].link_status < CORE_AT_LINK_CONN) {
		return STATE_AT_LINK_IS_DISCONN;
	}
	do {
		res = core_ringbuf_read(&at_handle.fd[socket_id].data_rb, buffer, len);
		if (res == len) {
			break;
		}
		at_sysdep->core_sysdep_sleep(AIOT_AT_RINGBUF_RETRY_INTERVAL);
		if (at_handle.fd[socket_id].link_status < CORE_AT_LINK_CONN) {
			return STATE_AT_LINK_IS_DISCONN;
	}
	} while ((at_sysdep->core_sysdep_time() - timeout_start) < timeout_ms);
	
	return res;
}

int32_t aiot_at_nwk_close(uint8_t socket_id)
{
	(void)socket_id;

	if (at_handle.is_init != 1) {
		return STATE_AT_NOT_INITED;
	}

	if (socket_id >= sizeof(at_handle.fd) / sizeof(at_handle.fd[0])) {
		return STATE_USER_INPUT_OUT_RANGE;
	}
	core_log(at_sysdep, 100, "aiot_at_nwk_close\r\n");
	core_at_commands_send_sync(&at_handle, at_disconn_cmd_list,
							    sizeof(at_disconn_cmd_list) / sizeof(core_at_cmd_item_t));
	/* reset ringbuf */
	core_ringbuf_deinit(&at_handle.fd[socket_id].data_rb);

	at_handle.fd[socket_id].link_status = CORE_AT_LINK_DISABLE;
	return STATE_SUCCESS;
}

int32_t aiot_at_uart_reception(uint8_t data)
{
	int32_t res = STATE_SUCCESS;

	if (at_handle.is_init != 1) {
		return STATE_AT_NOT_INITED;
	}

	switch (at_handle.sm.state) {
		case CORE_AT_READ_RSP: {
			if (at_handle.rsp_buf_offset >= (sizeof(at_handle.rsp_buf) - 1)) {
				memset(at_handle.rsp_buf, 0, sizeof(at_handle.rsp_buf));
				at_handle.rsp_buf_offset = 0;
				break;
			}

			at_handle.rsp_buf[at_handle.rsp_buf_offset++] = data;

			/* 检查是否为数据接口应答 */
			if (strstr(at_handle.rsp_buf, AT_RSP_TCP_RECV_HEAD)) {
				at_handle.rsp_buf[at_handle.rsp_buf_offset - 1] = '=';
				at_handle.sm.state = CORE_AT_PARSE_RECV_RSP;
				break;
			}

			/* 检查是否行结束'\n'或者为'>' */
			if ((at_handle.rsp_buf_offset > 2 && data == '\n') || data == '>') {
				res = core_ringbuf_write(&at_handle.rsp_rb, (uint8_t *)at_handle.rsp_buf, at_handle.rsp_buf_offset);
				//printf("[recv] [%llu] %s", at_sysdep->core_sysdep_time(), (uint8_t *)at_handle.rsp_buf);
				memset(at_handle.rsp_buf, 0, sizeof(at_handle.rsp_buf));
				at_handle.rsp_buf_offset = 0;
			}
		} break;
		/* 网络接收数据AT命令解析, 解析出link_id和接受数据长度, AT命令举例[+TCPRECV:0,10,1234567890] */
		case CORE_AT_PARSE_RECV_RSP: {
			if (at_handle.rsp_buf_offset >= (sizeof(at_handle.rsp_buf) - 1)) {
				memset(at_handle.rsp_buf, 0, sizeof(at_handle.rsp_buf));
				at_handle.rsp_buf_offset = 0;
				break;
			}

			at_handle.rsp_buf[at_handle.rsp_buf_offset] = data;

			if (data == '\n') {
				uint32_t first = strstr(at_handle.rsp_buf, ",") - at_handle.rsp_buf;

				if (first != at_handle.rsp_buf_offset) {
					uint64_t time = at_sysdep->core_sysdep_time();
					at_handle.sm.state = CORE_AT_READ_DATA;
					core_str2uint(at_handle.rsp_buf + first + 1, at_handle.rsp_buf_offset - first - 3, &at_handle.sm.data_len);
					core_str2uint(at_handle.rsp_buf + first - 1, 1, &at_handle.sm.curr_link_id);
					
          printf("[%llu][recv] +RECEIVE,%u,%u\r\n", at_sysdep->core_sysdep_time(), at_handle.sm.curr_link_id, at_handle.sm.data_len);
					memset(at_handle.rsp_buf, 0, sizeof(at_handle.rsp_buf));
					at_handle.rsp_buf_offset = 0;
					break;
				}
			}

			at_handle.rsp_buf_offset++;
		} break;
		case CORE_AT_READ_DATA: {
			res = core_ringbuf_write(&at_handle.fd[at_handle.sm.curr_link_id].data_rb, &data, 1);
			if (--at_handle.sm.data_len == 0) {
				at_handle.sm.state = CORE_AT_READ_RSP;
			}
		} break;
		default: break;
	}

	return res;
}

int32_t aiot_at_deinit(void)
{
	uint8_t i = 0;

	if (at_handle.is_init == 0) {
		return STATE_SUCCESS;
	}

	at_sysdep->core_sysdep_mutex_deinit(&at_handle.tx_mutex);

	for (i = 0; i < sizeof(at_handle.fd) / sizeof(at_handle.fd[0]); i++) {
		core_ringbuf_deinit(&at_handle.fd[i].data_rb);
	}

	core_ringbuf_deinit(&at_handle.rsp_rb);
	memset(&at_handle, 0, sizeof(core_at_handle_t));

	return STATE_SUCCESS;
}

