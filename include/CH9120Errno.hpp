/* @file CH9120Errno.hpp */

#pragma once

// ===================================================================================
// SOCKET ERROR CODES (Linux errno compatible)
// ===================================================================================

// Due to some headers including errno codes as defines (<errno.h>, <cerrno>...)
// we unfortunately cannot use constexpr or enums. In this case just use a raw
// define for each errno code.

#ifndef EAGAIN
    #define EAGAIN 11
#endif
#ifndef ENOMEM
    #define ENOMEM 12
#endif
#ifndef EACCES
    #define EACCES 13
#endif
#ifndef EFAULT
    #define EFAULT 14
#endif
#ifndef EBUSY
    #define EBUSY 16
#endif
#ifndef EINVAL
    #define EINVAL 22
#endif
#ifndef EMFILE
    #define EMFILE 24
#endif
#ifndef ENOSPC
    #define ENOSPC 28
#endif
#ifndef EPIPE
    #define EPIPE 32
#endif
#ifndef ENOBUFS
    #define ENOBUFS 105
#endif
#ifndef ENOTSOCK
    #define ENOTSOCK 108
#endif
#ifndef ECONNREFUSED
    #define ECONNREFUSED 111
#endif
#ifndef ENETUNREACH
    #define ENETUNREACH 114
#endif
#ifndef ENETDOWN
    #define ENETDOWN 115
#endif
#ifndef ETIMEDOUT
    #define ETIMEDOUT 116
#endif
#ifndef EHOSTDOWN
    #define EHOSTDOWN 117
#endif
#ifndef EHOSTUNREACH
    #define EHOSTUNREACH 118
#endif
#ifndef EINPROGRESS
    #define EINPROGRESS 119
#endif
#ifndef EALREADY
    #define EALREADY 120
#endif
#ifndef EMSGSIZE
    #define EMSGSIZE 122
#endif
#ifndef EISCONN
    #define EISCONN 127
#endif
#ifndef ENOTCONN
    #define ENOTCONN 128
#endif
#ifndef EWOULDBLOCK
    #define EWOULDBLOCK EAGAIN
#endif