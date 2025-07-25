/* @file CH9120Debug.hpp */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <cstdio>
#include <cstring>

// ===================================================================================
// LOG HELPERS
// ===================================================================================

namespace CH9120Debug {

/* @brief Context structure to capture call location information
 */
struct CallCtx {
    const char* file;
    const char* function;
    int line;
    
    CallCtx(const char* f = __builtin_FILE(), 
            const char* func = __builtin_FUNCTION(), 
            int l = __builtin_LINE()) 
        : file(f), function(func), line(l) {}
};

/* @brief Utility function to extract the filename from a full path
 * @param path The full path to extract the filename from
 * @return The filename
 */
static inline const char* getBasename(const char* path) {
    const char* basename = path;
    
    // Search for the last occurrence of '/' (Unix/Linux/macOS)
    const char* lastSlash = strrchr(path, '/');
    if (lastSlash) basename = lastSlash + 1;
    
    // Search for the last occurrence of '\' (Windows)
    const char* lastBackslash = strrchr(path, '\\');
    if (lastBackslash && lastBackslash > basename) basename = lastBackslash + 1;
    
    return basename;
}

#if defined(CH9120_DEBUG)
    /* @brief Log a formatted message with context information
     * @param ctx The call context
     * @param fmt The format string
     * @param args The arguments to format
     */
    template<typename... Args>
    void logf_ctx(CallCtx ctx, const char* fmt, Args... args) {
        printf("[%s::%s:%d] ", getBasename(ctx.file), ctx.function, ctx.line);
        printf(fmt, args...);
        printf("\n");
    }

    /* @brief Macro to log a formatted message with automatic context capture
     * @param format The format string
     * @param args The arguments to format
     */
    #define logf(format, ...) logf_ctx(CH9120Debug::CallCtx(), format, ##__VA_ARGS__)

    /* @brief Log a simple message with context information
     * @param msg The message to log
     * @param ctx The call context
     */
    inline void logln(const char* msg, CallCtx ctx = CallCtx()) {
        printf("[%s::%s:%d] %s\n", getBasename(ctx.file), ctx.function, ctx.line, msg);
    }
#else
    /* @brief No-op versions for release builds
     * @param args Catch-all
     */
    template<typename... Args>
    void logf_ctx(Args...) {}

    /* @brief No-op version for release builds
     * @param args Catch-all
     */
    template<typename... Args>
    void logf(Args...) {}

    /* @brief No-op version for release builds
     * @param args Catch-all
     */
    template<typename... Args>
    void logln(Args...) {}
#endif

} // namespace CH9120Debug