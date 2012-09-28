#ifndef _TRACE_SOCKET_H
#define _TRACE_SOCKET_H

#include <net/sock.h>
#include <linux/tracepoint.h>

DECLARE_TRACE(socket_create,
	TP_PROTO(int family, int type, int protocol, struct socket *sock,
	int ret),
	TP_ARGS(family, type, protocol, sock, ret));

DECLARE_TRACE(socket_bind,
	TP_PROTO(int fd, struct sockaddr __user *umyaddr, int addrlen, int ret),
	TP_ARGS(fd, umyaddr, addrlen, ret));

DECLARE_TRACE(socket_connect,
	TP_PROTO(int fd, struct sockaddr __user *uservaddr, int addrlen,
	int ret),
	TP_ARGS(fd, uservaddr, addrlen, ret));

DECLARE_TRACE(socket_listen,
	TP_PROTO(int fd, int backlog, int ret),
	TP_ARGS(fd, backlog, ret));

DECLARE_TRACE(socket_accept,
	TP_PROTO(int fd, struct sockaddr __user *upeer_sockaddr,
	int __user *upeer_addrlen, int flags, int ret),
	TP_ARGS(fd, upeer_sockaddr, upeer_addrlen, flags, ret));

DECLARE_TRACE(socket_getsockname,
	TP_PROTO(int fd, struct sockaddr __user *usockaddr,
	int __user *usockaddr_len, int ret),
	TP_ARGS(fd, usockaddr, usockaddr_len, ret));

DECLARE_TRACE(socket_getpeername,
	TP_PROTO(int fd, struct sockaddr __user *usockaddr,
	int __user *usockaddr_len, int ret),
	TP_ARGS(fd, usockaddr, usockaddr_len, ret));

DECLARE_TRACE(socket_socketpair,
	TP_PROTO(int family, int type, int protocol, int __user *usockvec,
	int ret),
	TP_ARGS(family, type, protocol, usockvec, ret));

DECLARE_TRACE(socket_sendmsg,
	TP_PROTO(struct socket *sock, struct msghdr *msg, size_t size, int ret),
	TP_ARGS(sock, msg, size, ret));

DECLARE_TRACE(socket_recvmsg,
	TP_PROTO(struct socket *sock, struct msghdr *msg, size_t size,
		int flags, int ret),
	TP_ARGS(sock, msg, size, flags, ret));

DECLARE_TRACE(socket_setsockopt,
	TP_PROTO(int fd, int level, int optname, char __user *optval,
	int optlen, int ret),
	TP_ARGS(fd, level, optname, optval, optlen, ret));

DECLARE_TRACE(socket_getsockopt,
	TP_PROTO(int fd, int level, int optname, char __user *optval,
	int __user *optlen, int ret),
	TP_ARGS(fd, level, optname, optval, optlen, ret));

DECLARE_TRACE(socket_shutdown,
	TP_PROTO(int fd, int how, int ret),
	TP_ARGS(fd, how, ret));

/*
 * socket_call
 *
 * We also trace socket_call so we can know which syscall is used by user
 * (socket_call or sock_send...)
 */
DECLARE_TRACE(socket_call,
	TP_PROTO(int call, unsigned long a0),
	TP_ARGS(call, a0));
#endif
