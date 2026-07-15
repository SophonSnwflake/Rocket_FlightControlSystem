#pragma once

#include "RSL_common.h"
#include <cmath>
#include <type_traits>
#include <array>
#include <Eigen/Dense>

template <typename T>
class Filter{
public:
    virtual ~Filter() = default;
    virtual T filterCalculate(T input) = 0;
    virtual void reset() = 0;
};


/**
 * @brief 一阶低通滤波器
 * @details 一阶低通滤波器，可用于平滑信号
 */
template <typename T = fp32>
class LowPassFilter : public Filter<T>
{
private:
    T m_alpha;      // 滤波系数 (0-1)
    T m_lastOutput; // 上一次的输出值

public:
    /**
     * @brief 构造函数
     * @param alpha 滤波系数 (0-1)，越接近1，滤波效果越弱
     */
    LowPassFilter(T alpha = 0.7f)
        : m_alpha(alpha), m_lastOutput(0)
    {
    }

    /**
     * @brief 滤波计算
     * @param input 输入值
     * @return 滤波后的输出值
     */
    T filterCalculate(T input) override
    {
        m_lastOutput = m_alpha * input + (1 - m_alpha) * m_lastOutput;
        return m_lastOutput;
    }

    /**
     * @brief 重置滤波器
     */
    void reset() override
    {
        m_lastOutput = 0;
    }

    /**
     * @brief 设置滤波系数
     * @param alpha 滤波系数 (0-1)
     */
    void setAlpha(T alpha)
    {
        m_alpha = alpha;
    }

    /**
     * @brief 获取滤波系数
     * @return 滤波系数
     */
    T getAlpha() const
    {
        return m_alpha;
    }
};


/**
 * @brief 移动平均滤波器
 * @details 使用一个固定大小的窗口计算平均值
 */
template <typename T = fp32, size_t WindowSize = 10>
class MovingAverageFilter : public Filter<T>
{
private:
    T m_buffer[WindowSize]; // 数据缓冲区
    size_t m_head;          // 缓冲区头指针
    size_t m_count;         // 当前缓冲区内的数据数量
    T m_sum;                // 当前所有数据的和

public:
    /**
     * @brief 构造函数
     */
    MovingAverageFilter()
        : m_head(0), m_count(0), m_sum(0)
    {
        for (size_t i = 0; i < WindowSize; i++) {
            m_buffer[i] = 0;
        }
    }

    /**
     * @brief 滤波计算
     * @param input 输入值
     * @return 滤波后的输出值
     */
    T filterCalculate(T input) override
    {
        // 更新总和，减去将被替换的值
        if (m_count == WindowSize) {
            m_sum -= m_buffer[m_head];
        } else {
            m_count++;
        }

        // 添加新值
        m_buffer[m_head] = input;
        m_sum += input;

        // 更新头指针
        m_head = (m_head + 1) % WindowSize;

        // 计算平均值
        return m_sum / static_cast<T>(m_count);
    }

    /**
     * @brief 重置滤波器
     */
    void reset() override
    {
        m_head  = 0;
        m_count = 0;
        m_sum   = 0;
        for (size_t i = 0; i < WindowSize; i++) {
            m_buffer[i] = 0;
        }
    }

    /**
     * @brief 获取窗口大小
     * @return 窗口大小
     */
    size_t getWindowSize() const
    {
        return WindowSize;
    }

    /**
     * @brief 获取当前缓冲区中的数据数量
     * @return 数据数量
     */
    size_t getCount() const
    {
        return m_count;
    }
};