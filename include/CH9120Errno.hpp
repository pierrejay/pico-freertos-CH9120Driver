/* @file CH9120Errno.hpp */

#pragma once

// ===================================================================================
// SOCKET ERROR CODES (Linux errno compatible)
// ===================================================================================

/* @brief Socket error codes (Linux errno compatible) */
namespace SockErrno {
    constexpr int EAGAIN = 11;           // Resource temporarily unavailable (would block)
    constexpr int ENOMEM = 12;           // Cannot allocate memory
    constexpr int EACCES = 13;           // Permission denied
    constexpr int EFAULT = 14;           // Bad address
    constexpr int EBUSY = 16;            // Device or resource busy
    constexpr int EINVAL = 22;           // Invalid argument
    constexpr int EMFILE = 24;           // Too many open files
    constexpr int ENOSPC = 28;           // No space left on device
    constexpr int EPIPE = 32;            // Broken pipe
    constexpr int ENOBUFS = 105;         // No buffer space available
    constexpr int ENOTSOCK = 108;        // Socket operation on non-socket
    constexpr int ECONNREFUSED = 111;    // Connection refused
    constexpr int ENETUNREACH = 114;     // Network is unreachable
    constexpr int ENETDOWN = 115;        // Network interface is not configured
    constexpr int ETIMEDOUT = 116;       // Connection timed out
    constexpr int EHOSTDOWN = 117;       // Host is down
    constexpr int EHOSTUNREACH = 118;    // Host is unreachable
    constexpr int EINPROGRESS = 119;     // Operation now in progress
    constexpr int EALREADY = 120;        // Operation already in progress
    constexpr int EMSGSIZE = 122;        // Message too long
    constexpr int ENOTCONN = 128;        // Transport endpoint is not connected
    constexpr int EISCONN = 127;         // Transport endpoint is already connected
    constexpr int EWOULDBLOCK = EAGAIN;  // Operation would block (alias)

} // namespace SockErrno