import numpy as np


def estimate_transform_2d(src_points, dst_points):
    """
    ===============================
    AMR1 Map → AMR2 Map 변환 행렬 계산
    ===============================

    src_points : AMR1에서 찍은 동일한 위치의 좌표들
    dst_points : AMR2에서 찍은 동일한 위치의 좌표들

    예)
    AMR1                         AMR2

    P1 -------- P2              P1 -------- P2
    |           |               |           |
    |           |      --->     |           |
    P3 -------- P4              P3 -------- P4

    같은 실제 위치를 두 로봇에서 각각 찍어준다.

    반환값
    --------------------
    R : 회전 행렬
    t : 평행이동 벡터
    T : 최종 3x3 변환 행렬
    """

    # 리스트를 numpy 배열로 변환
    src = np.array(src_points, dtype=np.float64)
    dst = np.array(dst_points, dtype=np.float64)

    # 좌표 개수가 같은지 확인
    assert src.shape == dst.shape

    # 최소 두 점 이상 필요
    assert src.shape[0] >= 2

    # x,y 좌표만 사용
    assert src.shape[1] == 2

    # ----------------------------------
    # 각 좌표 집합의 중심점(평균 좌표) 계산
    # ----------------------------------
    src_centroid = np.mean(src, axis=0)
    dst_centroid = np.mean(dst, axis=0)

    # 중심을 원점(0,0)으로 이동
    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid

    # ----------------------------------
    # SVD를 이용하여
    # 두 좌표계의 최적 회전 계산
    # ----------------------------------
    H = src_centered.T @ dst_centered
    U, S, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    # 반사(거울 대칭) 행렬이 나오면 수정
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # ----------------------------------
    # 평행이동 계산
    #
    # AMR1 좌표를 회전시킨 후
    # 얼마나 이동해야 AMR2와 맞는지 계산
    # ----------------------------------
    t = dst_centroid - R @ src_centroid

    # ----------------------------------
    # 최종 3x3 Homogeneous Transform 생성
    #
    #      ┌             ┐
    #      │ R11 R12 tx │
    #  T = │ R21 R22 ty │
    #      │  0   0   1 │
    #      └             ┘
    # ----------------------------------
    T = np.eye(3)

    T[:2, :2] = R
    T[:2, 2] = t

    return R, t, T


def transform_point(point, T):
    """
    ===============================
    좌표 하나 변환
    ===============================

    AMR1 좌표 하나를

    ↓

    AMR2 좌표로 변환

    예)

    (-5.2, 1.0)

    ↓

    (-6.8, 0.3)
    """

    # [x,y] → [x,y,1]
    p = np.array([point[0], point[1], 1.0])

    # 변환 행렬 적용
    p2 = T @ p

    return [
        float(p2[0]),
        float(p2[1])
    ]


def print_error(src_points, dst_points, T):
    """
    ===============================
    변환 정확도 확인
    ===============================

    입력한 기준 좌표들이

    실제 좌표와 얼마나 차이나는지 출력

    평균 오차
    최대 오차

    를 확인할 수 있다.
    """

    src = np.array(src_points, dtype=np.float64)
    dst = np.array(dst_points, dtype=np.float64)

    errors = []

    print("\n===== 변환 오차 확인 =====")

    for i, (s, d) in enumerate(zip(src, dst), start=1):

        # 계산된 좌표
        pred = transform_point(s, T)

        # 실제 좌표와의 거리 오차(m)
        err = np.linalg.norm(np.array(pred) - d)

        errors.append(err)

        print(
            f"P{i}: "
            f"AMR1 {s.tolist()} "
            f"→ 변환 {pred} "
            f"| AMR2 실제 {d.tolist()} "
            f"| 오차 {err:.3f} m"
        )

    print(f"\n평균 오차 : {np.mean(errors):.3f} m")
    print(f"최대 오차 : {np.max(errors):.3f} m")


# ====================================================
# 같은 실제 위치를
#
# AMR1에서 찍은 좌표
#
# AMR2에서 찍은 좌표
#
# 순서를 동일하게 입력
# ====================================================

amr1_points = [
    # [x, y],
]

amr2_points = [
    # [x, y],
]


# ====================================================
# 변환 행렬 계산
# ====================================================

R, t, T = estimate_transform_2d(
    amr1_points,
    amr2_points
)

print("===== 회전 행렬(R) =====")
print(R)

print("\n===== 평행이동(t) =====")
print(t)

print("\n===== 최종 변환 행렬(T) =====")
print(T)

# ====================================================
# 입력했던 기준 좌표들의
# 오차 확인
# ====================================================

print_error(
    amr1_points,
    amr2_points,
    T
)

# ====================================================
# 실제 사용할 좌표 변환 예시
#
# AMR1이 발견한 좌표
#
# ↓
#
# AMR2 좌표로 변환
# ====================================================

problem_point_amr1 = [
    -5.2,
    1.0
]

converted = transform_point(
    problem_point_amr1,
    T
)

print("\n===== 새 좌표 변환 =====")

print(
    f"AMR1 좌표 {problem_point_amr1}"
)

print(
    f"AMR2 좌표 {converted}"
)