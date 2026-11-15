#include "xsting_approach/mlp.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>

namespace xsting_approach {

namespace {

// 지원 가중치 매직 헤더 및 ELU 활성화 함수
constexpr char kMagicXst[8] = {'X', 'S', 'T', 'M', 'L', 'P', '0', '1'};
constexpr char kMagicMk2[8] = {'M', 'K', '2', 'M', 'L', 'P', '0', '1'};

inline float CalcElu(float x) { return x >= 0.0f ? x : (std::exp(x) - 1.0f); }

}  // namespace

// 이진 파일에서 정규화 통계·레이어 가중치·추가 벡터 읽기
bool Mlp::Load(const std::string& path, std::string* err) {
  loaded_ = false;
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    if (err) *err = "가중치 파일 열기 실패: " + path;
    return false;
  }

  // 매직 헤더 및 메타데이터 검증
  char magic[8]{};
  ifs.read(magic, 8);
  const bool magic_ok = (std::memcmp(magic, kMagicXst, 8) == 0) ||
                        (std::memcmp(magic, kMagicMk2, 8) == 0);
  uint32_t dim = 0;
  uint32_t num_layers = 0;
  uint32_t num_extra = 0;
  ifs.read(reinterpret_cast<char*>(&dim), sizeof(uint32_t));
  ifs.read(reinterpret_cast<char*>(&num_layers), sizeof(uint32_t));
  ifs.read(reinterpret_cast<char*>(&num_extra), sizeof(uint32_t));
  ifs.read(reinterpret_cast<char*>(&clip_obs_), sizeof(float));
  if (!ifs || !magic_ok || dim == 0 || num_layers == 0 || num_layers > 16) {
    if (err) *err = "잘못된 MLP 헤더: " + path;
    return false;
  }

  // 입력 평균·표준편차 읽기
  mean_.resize(dim);
  std_.resize(dim);
  ifs.read(reinterpret_cast<char*>(mean_.data()), sizeof(float) * dim);
  ifs.read(reinterpret_cast<char*>(std_.data()), sizeof(float) * dim);

  // 각 선형 계층 행우선 가중치 및 편향 읽기
  layers_.clear();
  layers_.reserve(num_layers);
  uint32_t prev_cols = dim;
  for (uint32_t i = 0; i < num_layers; ++i) {
    uint32_t rows = 0;
    uint32_t cols = 0;
    ifs.read(reinterpret_cast<char*>(&rows), sizeof(uint32_t));
    ifs.read(reinterpret_cast<char*>(&cols), sizeof(uint32_t));
    if (!ifs || rows == 0 || cols != prev_cols) {
      if (err) *err = "레이어 차원 불일치: " + path;
      return false;
    }

    std::vector<float> buf(static_cast<size_t>(rows) * cols);
    ifs.read(reinterpret_cast<char*>(buf.data()), sizeof(float) * buf.size());
    Layer layer;
    layer.w = Eigen::Map<
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        buf.data(), rows, cols);
    layer.b.resize(rows);
    ifs.read(reinterpret_cast<char*>(layer.b.data()), sizeof(float) * rows);
    if (!ifs) {
      if (err) *err = "가중치 버퍼 읽기 실패: " + path;
      return false;
    }
    layers_.push_back(std::move(layer));
    prev_cols = rows;
  }

  // 기본 관절각 등 추가 벡터 읽기
  extras_.clear();
  extras_.reserve(num_extra);
  for (uint32_t i = 0; i < num_extra; ++i) {
    uint32_t len = 0;
    ifs.read(reinterpret_cast<char*>(&len), sizeof(uint32_t));
    Eigen::VectorXf vec(len);
    if (len > 0) {
      ifs.read(reinterpret_cast<char*>(vec.data()), sizeof(float) * len);
    }
    if (!ifs) {
      if (err) *err = "추가 벡터 읽기 실패: " + path;
      return false;
    }
    extras_.push_back(std::move(vec));
  }

  loaded_ = true;
  return true;
}

// 입력 정규화 후 ELU 은닉층과 [-1, 1] 출력 클리핑 수행
Eigen::VectorXf Mlp::Forward(const Eigen::Ref<const Eigen::VectorXf>& obs) const {
  Eigen::VectorXf x = (obs - mean_).cwiseQuotient(std_);
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    const float val = std::isfinite(x[i]) ? x[i] : 0.0f;
    x[i] = std::clamp(val, -clip_obs_, clip_obs_);
  }

  for (size_t l = 0; l < layers_.size(); ++l) {
    x = layers_[l].w * x + layers_[l].b;
    if (l + 1 < layers_.size()) {
      for (Eigen::Index i = 0; i < x.size(); ++i) x[i] = CalcElu(x[i]);
    } else {
      for (Eigen::Index i = 0; i < x.size(); ++i) {
        x[i] = std::clamp(x[i], -1.0f, 1.0f);
      }
    }
  }
  return x;
}

}  // namespace xsting_approach
