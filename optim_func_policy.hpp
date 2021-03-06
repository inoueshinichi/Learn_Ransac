#pragma once

#include "format_string.hpp"

#include <vector>
#include <tuple>
#include <map>

class RansacLine 
{
    using candidate_currect_models = std::map<double, std::tuple<double,double>>;
    candidate_currect_models models_;

public:
    // 直線の式: ax + b, パラメータ: a, b
    using params_type = std::tuple<double, double>; 
    
    RansacLine() {}
    virtual ~RansacLine() {}

    template <class DataType>
    params_type estimate_params(const std::vector<DataType>& selected_data)
    {
        // Concept
        static_assert(std::is_same<DataType, std::pair<double,double>>::value, 
                      "DataType must be std::pair<double,double>.");

        double x = 0;
        double y = 0;
        double sum_x = 0;
        double sum_xx = 0;
        double sum_xy = 0;
        double sum_y = 0;
        double num = 0;

        for (auto&& point : selected_data) {
            x = point.first;
            y = point.second;
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
            num += 1;
        }
        /*公式*/
        double a_numerator = num * sum_xy - sum_x * sum_y;
        double b_numerator = sum_xx * sum_y - sum_xy * sum_x;
        double denominator = num * sum_xx - sum_x * sum_x;
        double a = a_numerator / denominator;
        double b = b_numerator / denominator;

        return std::make_tuple(a, b);
    }

    template <class DataType>
    double calculate_error(const params_type& params, const DataType& point)
    {
        // Concept
        static_assert(std::is_same<DataType, std::pair<double,double>>::value, 
                      "DataType must be std::pair<double,double>.");

        double a = std::get<0>(params);
        double b = std::get<1>(params);
        double x = point.first;
        double y = point.second;
        double ef = a * x + b; // 直線の推定値
        return y - ef; // 符号あり誤差
    }

    template <class DataType>
    double calculate_model_error(const params_type& params, const std::vector<DataType>& data)
    {
        double model_error = 0.0; // 各データ点における二乗誤差の和
        for (auto&& point : data) {
            double error = calculate_error(params, point); // 符号付き誤差
            model_error += 0.5 * error * error; // 二乗誤差
        }
        return model_error;
    }

    void set_candidate_model(double model_error, const params_type& params)
    {
        // model_errorは二乗誤差
        models_.insert(std::make_pair(model_error, params)); // std::mapなので自動で誤差値に基づいて昇順ソートされる
    }

    params_type& select_optimized_params()
    {
        return std::begin(models_)->second;
    }

};