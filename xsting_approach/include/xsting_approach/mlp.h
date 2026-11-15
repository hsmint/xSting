#ifndef XSTING_APPROACH_MLP_H_
#define XSTING_APPROACH_MLP_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace xsting_approach {

// 입력 정규화 및 ELU 은닉층을 포함한 경량 MLP 추론기
class Mlp {
 public:
  // 이진 가중치 파일(XSTMLP01/MK2MLP01) 로드
  bool Load(const std::string& path, std::string* err = nullptr);

  // 관측 벡터로부터 [-1, 1] 범위 행동 추론
  Eigen::VectorXf Forward(const Eigen::Ref<const Eigen::VectorXf>& obs) const;

  // 입출력 차원 및 부가 벡터 조회
  bool loaded() const { return loaded_; }
  int in_dim() const { return static_cast<int>(mean_.size()); }
  int out_dim() const {
    return layers_.empty() ? 0 : static_cast<int>(layers_.back().b.size());
  }
  const std::vector<Eigen::VectorXf>& extras() const { return extras_; }

 private:
  struct Layer {
    Eigen::MatrixXf w;
    Eigen::VectorXf b;
  };

  bool loaded_ = false;
  float clip_obs_ = 5.0f;
  Eigen::VectorXf mean_;
  Eigen::VectorXf std_;
  std::vector<Layer> layers_;
  std::vector<Eigen::VectorXf> extras_;
};

}  // namespace xsting_approach

#endif  // XSTING_APPROACH_MLP_H_
