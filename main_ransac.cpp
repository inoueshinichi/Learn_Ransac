#include "ransac.hpp"

#include <iostream>
#include <random>

int main(int, char**) {
    std::cout << "Hello, Ransac!\n";

    uint32_t data_num = 20;
    uint32_t max_iter = 800;
    double thresh = 1.0;
    int required_inlier = 80;
    uint64_t seed = 20210306;
    Ransac<RansacLine> ransac(data_num, max_iter, thresh, required_inlier, seed);

    // ダミーデータ準備
    std::mt19937_64 engine(seed);
    std::normal_distribution<double> normal_dist(0.0, 1.0); // 平均：0.0, 標準偏差1.0
    std::uniform_int_distribution<> uniform_dist(0, 10); // 0~10
    double a = 0.5;
    double b = 0.3;
    std::vector<std::pair<double, double>> data;
    for (int i = 0; i < 100; ++i) {
        double x = i * 0.1;
        double y = a * x + b + normal_dist(engine) + (uniform_dist(engine) == 0 ? 1 : 0); // モデル＋ゆらぎ＋ノイズ
        data.emplace_back(std::make_pair(x, y));
    }
    for (int index = 0; const auto& point : data) {
        std::cout << "index: " << index << " -> (x, y): " << 
        "(" << point.first << "," << point.second << ")" << std::endl;
        index += 1;
    }

    // // static_assertチェック用ダミーデータ
    // std::vector<int> data;
    // for (int i = 0; i < 100; ++i) {
    //     data.push_back(i*10);
    // }

    // Ransacを実行
    auto params = ransac.execute(data);

    std::cout << "直線の最適パラメータ: a=" << std::get<0>(params) << ", b=" << std::get<1>(params) << std::endl;

    std::cout << "正解候補のモデル達" << std::endl;
    auto candidate_models = ransac.get_candidate_models();
    for (auto&& model : candidate_models)
    {
        std::cout << "error: " << model.first << 
        ", a: " << std::get<0>(model.second) << ", b: " << std::get<1>(model.second) << std::endl;
    }
}
