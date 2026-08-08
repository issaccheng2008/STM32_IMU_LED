#!/usr/bin/env python3
"""Refit saved LSM6DSV320X calibration points using ST DT0059.

This standard-library-only tool mirrors the firmware's numerically improved
rotated-ellipsoid fit and prints paste-ready C constants.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Sequence

GRAVITY_MG = 1000.0
PARAMETER_COUNT = 9
EPSILON = 1.0e-12


class FitError(RuntimeError):
    """Raised when the saved points do not define a valid ellipsoid."""


def solve_linear(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> list[float]:
    size = len(vector)
    augmented = [list(matrix[row]) + [vector[row]] for row in range(size)]

    for column in range(size):
        pivot_row = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot_row][column]) < EPSILON:
            raise FitError("singular least-squares system; use more widely distributed points")
        augmented[column], augmented[pivot_row] = augmented[pivot_row], augmented[column]

        pivot = augmented[column][column]
        augmented[column] = [value / pivot for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                augmented[row][item] - factor * augmented[column][item]
                for item in range(size + 1)
            ]

    return [augmented[row][size] for row in range(size)]


def symmetric_eigen_3(
    matrix_in: Sequence[Sequence[float]],
) -> tuple[list[float], list[list[float]]]:
    matrix = [list(row) for row in matrix_in]
    vectors = [[1.0 if row == column else 0.0 for column in range(3)] for row in range(3)]

    for _ in range(32):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda pair: abs(matrix[pair[0]][pair[1]]))
        if abs(matrix[p][q]) < EPSILON:
            break

        app, aqq, apq = matrix[p][p], matrix[q][q], matrix[p][q]
        tau = (aqq - app) / (2.0 * apq)
        tangent = (1.0 if tau >= 0.0 else -1.0) / (abs(tau) + math.sqrt(1.0 + tau * tau))
        cosine = 1.0 / math.sqrt(1.0 + tangent * tangent)
        sine = tangent * cosine

        for index in range(3):
            if index in (p, q):
                continue
            aip, aiq = matrix[index][p], matrix[index][q]
            matrix[index][p] = matrix[p][index] = cosine * aip - sine * aiq
            matrix[index][q] = matrix[q][index] = sine * aip + cosine * aiq

        matrix[p][p] = cosine * cosine * app - 2.0 * sine * cosine * apq + sine * sine * aqq
        matrix[q][q] = sine * sine * app + 2.0 * sine * cosine * apq + cosine * cosine * aqq
        matrix[p][q] = matrix[q][p] = 0.0

        for row in range(3):
            vip, viq = vectors[row][p], vectors[row][q]
            vectors[row][p] = cosine * vip - sine * viq
            vectors[row][q] = sine * vip + cosine * viq

    return [matrix[index][index] for index in range(3)], vectors


def fit_rotated_ellipsoid(
    points_mg: Sequence[Sequence[float]],
) -> tuple[list[float], list[list[float]], float]:
    if len(points_mg) < PARAMETER_COUNT:
        raise FitError(f"at least {PARAMETER_COUNT} non-coplanar points are required")

    normal = [[0.0] * PARAMETER_COUNT for _ in range(PARAMETER_COUNT)]
    rhs_normal = [0.0] * PARAMETER_COUNT

    for raw_x, raw_y, raw_z in points_mg:
        x, y, z = raw_x / GRAVITY_MG, raw_y / GRAVITY_MG, raw_z / GRAVITY_MG
        x2, y2, z2 = x * x, y * y, z * z
        row = [
            x2 + y2 - 2.0 * z2,
            x2 - 2.0 * y2 + z2,
            4.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            2.0 * x,
            2.0 * y,
            2.0 * z,
            1.0,
        ]
        rhs = x2 + y2 + z2
        for row_index in range(PARAMETER_COUNT):
            rhs_normal[row_index] += row[row_index] * rhs
            for column_index in range(PARAMETER_COUNT):
                normal[row_index][column_index] += row[row_index] * row[column_index]

    alternative = solve_linear(normal, rhs_normal)
    transformed = [
        -1.0 + alternative[0] + alternative[1],
        -1.0 + alternative[0] - 2.0 * alternative[1],
        -1.0 - 2.0 * alternative[0] + alternative[1],
        2.0 * alternative[2],
        alternative[3],
        alternative[4],
        alternative[5],
        alternative[6],
        alternative[7],
        alternative[8],
    ]
    if abs(transformed[9]) < EPSILON:
        raise FitError("fit normalization is singular")
    coefficients = [-value / transformed[9] for value in transformed[:9]]

    quadratic = [
        [coefficients[0], coefficients[3], coefficients[4]],
        [coefficients[3], coefficients[1], coefficients[5]],
        [coefficients[4], coefficients[5], coefficients[2]],
    ]
    center = solve_linear(quadratic, [-value for value in coefficients[6:9]])
    centered_scale = 1.0 + sum(
        center[row] * quadratic[row][column] * center[column]
        for row in range(3)
        for column in range(3)
    )
    if centered_scale <= EPSILON:
        raise FitError("quadratic is not a real ellipsoid")

    translated = [[value / centered_scale for value in row] for row in quadratic]
    eigenvalues, eigenvectors = symmetric_eigen_3(translated)
    if any(value <= EPSILON for value in eigenvalues):
        raise FitError("quadratic is not positive definite")

    correction = [
        [
            sum(
                eigenvectors[row][axis]
                * math.sqrt(eigenvalues[axis])
                * eigenvectors[column][axis]
                for axis in range(3)
            )
            for column in range(3)
        ]
        for row in range(3)
    ]
    offsets_mg = [value * GRAVITY_MG for value in center]

    squared_errors = []
    for point in points_mg:
        centered = [point[axis] - offsets_mg[axis] for axis in range(3)]
        corrected = [
            sum(correction[row][column] * centered[column] for column in range(3))
            for row in range(3)
        ]
        magnitude = math.sqrt(sum(value * value for value in corrected))
        squared_errors.append((magnitude - GRAVITY_MG) ** 2)
    rms_mg = math.sqrt(sum(squared_errors) / len(squared_errors))
    return offsets_mg, correction, rms_mg


def load_csv(path: Path) -> tuple[list[list[float]], list[list[float]], list[float] | None]:
    lines = path.read_text(encoding="utf-8").splitlines()
    gyro = None
    for line in lines:
        if line.startswith("# gyro_offset_dps,"):
            gyro = [float(value) for value in line.split(",")[1:4]]

    rows = list(csv.DictReader(line for line in lines if line and not line.startswith("#")))
    low_g = [[float(row[f"low_g_{axis}_mg"]) for axis in "xyz"] for row in rows]
    high_g = [[float(row[f"high_g_{axis}_mg"]) for axis in "xyz"] for row in rows]
    return low_g, high_g, gyro


def accel_initializer(
    name: str,
    offsets: Sequence[float],
    matrix: Sequence[Sequence[float]],
) -> str:
    values = {
        "offset_x_mg": offsets[0],
        "offset_y_mg": offsets[1],
        "offset_z_mg": offsets[2],
        "x_gain": matrix[0][0],
        "y_to_x": matrix[0][1],
        "z_to_x": matrix[0][2],
        "x_to_y": matrix[1][0],
        "y_gain": matrix[1][1],
        "z_to_y": matrix[1][2],
        "x_to_z": matrix[2][0],
        "y_to_z": matrix[2][1],
        "z_gain": matrix[2][2],
    }
    body = "\n".join(f"    .{field} = {value:.9f}f," for field, value in values.items())
    return f"const imu_accel_calibration_t {name} = {{\n{body}\n}};"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path, help="CSV saved by the firmware calibration session")
    args = parser.parse_args()

    low_g_points, high_g_points, gyro = load_csv(args.csv_file)
    low_offsets, low_matrix, low_rms = fit_rotated_ellipsoid(low_g_points)
    high_offsets, high_matrix, high_rms = fit_rotated_ellipsoid(high_g_points)

    print(f"Low-g RMS magnitude error:  {low_rms:.3f} mg")
    print(f"High-g RMS magnitude error: {high_rms:.3f} mg\n")
    print(accel_initializer("imu_low_g_calibration", low_offsets, low_matrix))
    print()
    print(accel_initializer("imu_high_g_calibration", high_offsets, high_matrix))
    if gyro is not None:
        print("\nconst imu_gyro_calibration_t imu_gyro_calibration = {")
        print(f"    .gyro_offset_x = {gyro[0]:.9f}f,")
        print(f"    .gyro_offset_y = {gyro[1]:.9f}f,")
        print(f"    .gyro_offset_z = {gyro[2]:.9f}f,")
        print("};")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
