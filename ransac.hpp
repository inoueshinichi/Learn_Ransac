#pragma once

#include "optim_func_policy.hpp"

#include <tuple>
#include <vector>
#include <random>

template <class OptimFunc>
class Ransac
{
    OptimFunc optim_func_; // 直線、二次曲線、楕円などRansac対象の数式モデル
    uint32_t data_num_; // ランダムに選択されるデータ数
    uint32_t max_iter_;
    double thresh_;
    uint32_t required_inliers_;
    std::uint_fast64_t seed_;
    std::mt19937_64 engine_;
    using dist_type = std::uniform_int_distribution<>;
public:
    Ransac() = delete;
    ~Ransac() = default;
    Ransac(const Ransac&) = default;
    Ransac& operator=(const Ransac&) = default;
    Ransac(Ransac&&) = default;
    Ransac& operator=(Ransac&&) = default;

    template <class... Args>
    Ransac(uint32_t data_num, uint32_t max_iter, double thresh, uint32_t required_inliers, uint64_t seed, Args... args)
        : optim_func_(args...)
        , data_num_(data_num)
        , max_iter_(max_iter)
        , thresh_(thresh)
        , required_inliers_(required_inliers)
        , seed_(seed)
        , engine_(seed_) {}
        
    template <class T>
    typename OptimFunc::params_type execute(const std::vector<T>& data)
    {
        // 一様分布の準備
        std::size_t data_size = data.size();
        dist_type dist(0, data_size - 1); // 0 ~ data_size - 1 のインデックスをランダム出力

        // Ransacイテレーション
        for (int iter = 0; iter < max_iter_; ++iter) {
            // ランダムにデータを選ぶ
            std::vector<T> selected_data;
            for (int i = 0; i < data_num_; ++i) {
                int index = dist(engine_);
                selected_data.push_back(data.at(index)); // 同じデータが選ばれても受け入れる仕様
            }
            
            // 選択したデータから関数のパラメータを推定
            auto params = optim_func_.estimate_params(selected_data);
            double error = 0.0;
            std::vector<T> inliers;
            for (int index = 0; index < data_size; ++index) {
                // 各データ点の値と推定値の誤差を計算
                error = optim_func_.calculate_error(params, data.at(index));

                // 誤差がしきい値以下だった場合、この時のデータ点をインライアとして登録
                if (error <= thresh_) {
                    inliers.push_back(data.at(index));
                }
            }

            // 登録されたインライアが要求点数を超えていたら、
            // その時の推定モデル(パラメータとモデル誤差)を
            // 正解候補モデルに登録
            if (inliers.size() > required_inliers_) {
                double model_error = optim_func_.calculate_model_error(params, data);
                optim_func_.set_candidate_model(model_error, params);
            }
        }

        // 正しいモデル候補から最小誤差に対応するモデルパラメータを選択
        auto optimized_params = optim_func_.select_optimized_params();

        return optimized_params;
    }

    typename OptimFunc::candidate_currect_models& get_candidate_models()
    {
        return optim_func_.get_candidate_models();
    }
};