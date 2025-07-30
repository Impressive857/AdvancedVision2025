#ifndef _MODEL_BASE_HPP_
#define _MODEL_BASE_HPP_

// acl
#include "acl/acl.h"

// std
#include <string>

template <typename _Resource, typename _Result>
class ModelBase {
public:
    ModelBase() = default;
    virtual bool init() = 0;
    bool load_model(const std::string& model_path) {
        return _load_model(model_path);
    }
    virtual _Result inference(const _Resource& resource) = 0;
    bool unload_model() {
        return _unload_model();
    }
    virtual bool finalize() = 0;
    virtual ~ModelBase() = default;
private:
    bool _load_model(const std::string& model_path) {
        size_t model_work_size, model_weight_size;
        aclError ret = aclmdlQuerySize(model_path.c_str(), &model_work_size, &model_weight_size);
        if (ACL_SUCCESS != ret) {
            return false;
        }

        ret = aclrtMalloc(&m_model_work_ptr, model_work_size, ACL_MEM_MALLOC_HUGE_FIRST);
        if (ACL_SUCCESS != ret) {
            return false;
        }

        ret = aclrtMalloc(&m_model_weight_ptr, model_weight_size, ACL_MEM_MALLOC_HUGE_FIRST);
        if (ACL_SUCCESS != ret) {
            aclrtFree(m_model_work_ptr);
            m_model_work_ptr = nullptr;
            return false;
        }

        ret = aclmdlLoadFromFileWithMem(model_path.c_str(), &m_model_id, m_model_work_ptr, model_work_size, m_model_weight_ptr, model_weight_size);
        if (ACL_SUCCESS != ret) {
            return false;
        }

        m_model_description = aclmdlCreateDesc();
        ret = aclmdlGetDesc(m_model_description, m_model_id);
        if (ACL_SUCCESS != ret) {
            return false;
        }

        return _prepare_dataset();
    }

    bool _prepare_dataset() {
        m_input_dataset = aclmdlCreateDataset();
        if (nullptr == m_input_dataset) {
            return false;
        }

        m_output_dataset = aclmdlCreateDataset();
        if (nullptr == m_output_dataset) {
            return false;
        }

        size_t input_num = aclmdlGetNumInputs(m_model_description);
        if (input_num < 1) {
            return false;
        }

        for (size_t i = 0; i < input_num; ++i) {
            size_t input_size = aclmdlGetInputSizeByIndex(m_model_description, i);
            if (input_size <= 0) {
                return false;
            }

            void* input_buf = nullptr;
            aclrtMalloc(&input_buf, input_size, ACL_MEM_MALLOC_HUGE_FIRST);

            if (nullptr == input_buf) {
                return false;
            }

            aclDataBuffer* input_data_buf = aclCreateDataBuffer(input_buf, input_size);
            if (nullptr == input_data_buf) {
                aclrtFree(input_buf);
                return false;
            }
            aclError ret = aclmdlAddDatasetBuffer(m_input_dataset, input_data_buf);
            if (ACL_SUCCESS != ret) {
                aclDestroyDataBuffer(input_data_buf);
                aclrtFree(input_buf);
                return false;
            }
        }

        size_t output_num = aclmdlGetNumOutputs(m_model_description);
        for (size_t i = 0; i < output_num; ++i) {
            size_t output_size = aclmdlGetOutputSizeByIndex(m_model_description, i);
            if (output_size <= 0) {
                return false;
            }

            void* output_buf = nullptr;
            aclrtMalloc(&output_buf, output_size, ACL_MEM_MALLOC_HUGE_FIRST);
            if (nullptr == output_buf) {
                return false;
            }

            aclDataBuffer* output_data_buf = aclCreateDataBuffer(output_buf, output_size);
            if (nullptr == output_data_buf) {
                aclrtFree(output_buf);
                return false;
            }

            aclError ret = aclmdlAddDatasetBuffer(m_output_dataset, output_data_buf);
            if (ACL_SUCCESS != ret) {
                aclDestroyDataBuffer(output_data_buf);
                aclrtFree(output_buf);
                return false;
            }
        }

        return true;
    }

    bool _unload_model() {
        bool success = _destory_dataset();
        if (!success) {
            return false;
        }

        if (nullptr != m_model_description) {
            aclmdlDestroyDesc(m_model_description);
            m_model_description = nullptr;
        }

        if (0 != m_model_id) {
            aclmdlUnload(m_model_id);
            m_model_id = 0;
        }

        if (nullptr != m_model_work_ptr) {
            aclrtFree(m_model_work_ptr);
            m_model_work_ptr = nullptr;
        }

        if (nullptr != m_model_weight_ptr) {
            aclrtFree(m_model_weight_ptr);
            m_model_weight_ptr = nullptr;
        }

        return true;
    }

    bool _destory_dataset() {
        if (nullptr != m_input_dataset) {
            for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(m_input_dataset); ++i) {
                aclDataBuffer* data_buffer = aclmdlGetDatasetBuffer(m_input_dataset, i);
                void* data = aclGetDataBufferAddr(data_buffer);
                if (data) aclrtFree(data);
                aclDestroyDataBuffer(data_buffer);
            }
            aclmdlDestroyDataset(m_input_dataset);
            m_input_dataset = nullptr;
        }

        if (nullptr != m_output_dataset) {
            for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(m_output_dataset); ++i) {
                aclDataBuffer* data_buffer = aclmdlGetDatasetBuffer(m_output_dataset, i);
                void* data = aclGetDataBufferAddr(data_buffer);
                if (data) aclrtFree(data);
                aclDestroyDataBuffer(data_buffer);
            }
            aclmdlDestroyDataset(m_output_dataset);
            m_output_dataset = nullptr;
        }

        return true;
    }
protected:
    aclmdlDataset* m_input_dataset = nullptr;
    aclmdlDataset* m_output_dataset = nullptr;
    aclmdlDesc* m_model_description = nullptr;
    uint32_t m_model_id = 0;
private:
    void* m_model_work_ptr = nullptr;
    void* m_model_weight_ptr = nullptr;
};

#endif // !_MODEL_BASE_HPP