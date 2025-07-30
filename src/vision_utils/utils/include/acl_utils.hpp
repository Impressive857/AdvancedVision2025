#ifndef _ACL_UTILS_HPP_
#define _ACL_UTILS_HPP_

// acl
#include <acl/acl.h>

struct acl_utils {
    static const int32_t decive_id = 0;
    static aclrtContext model_context;
    static aclrtStream model_stream;

    static bool init_acl() {
        aclError ret = aclInit(nullptr);
        if (ACL_SUCCESS != ret) {
            return false;
        }

        ret = aclrtSetDevice(decive_id);
        if (ACL_SUCCESS != ret) {
            return false;
        }

        ret = aclrtCreateContext(&model_context, decive_id);
        if (ACL_SUCCESS != ret) {
            return false;
        }

        ret = aclrtCreateStream(&model_stream);
        if (ACL_SUCCESS != ret) {
            return false;
        }

        return true;
    }

    static bool finalize_acl() {
        aclError ret;
        if (nullptr != model_stream) {
            ret = aclrtDestroyStream(model_stream);
            if (ACL_SUCCESS != ret) {
                return false;
            }
        }

        if (nullptr != model_context) {
            ret = aclrtDestroyContext(model_context);
            if (ACL_SUCCESS != ret) {
                return false;
            }
            model_context = nullptr;
        }

        ret = aclrtResetDevice(decive_id);
        if (ACL_SUCCESS != ret) {
            return false;
        }

        ret = aclFinalize();
        if (ACL_SUCCESS != ret) {
            return false;
        }

        return true;
    }
};

#endif // !_ACL_UTILS_HPP_