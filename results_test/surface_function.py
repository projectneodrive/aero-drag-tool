from __future__ import annotations

import numpy as np

DEFAULT_BIAS = 0.8423609598534698
DEFAULT_WEIGHTS = np.array([2.85312342e-36, 2.26254379e-01, 1.95785026e-01, 2.69151587e-01, 1.76207116e-19, 1.23485176e-01, 7.61088685e-35,
 1.18094717e-01, 7.55633078e-03, 2.32984136e-01, 1.56183518e-01, 2.59619186e-01], dtype=float)
CENTERS = np.array([[ 0.39965263,  0.91666667,  0.        ],
 [-0.48772367,  0.75      ,  0.44679483],
 [ 0.07101005,  0.58333333, -0.80912286],
 [ 0.55310703,  0.41666667,  0.72143018],
 [-0.95344473,  0.25      , -0.16865095],
 [ 0.84082048,  0.08333333, -0.53486117],
 [-0.25870133, -0.08333333,  0.96235606],
 [-0.44627131, -0.25      , -0.85926825],
 [ 0.8538988 , -0.41666667,  0.31184247],
 [-0.75078384, -0.58333333,  0.30991265],
 [ 0.28034777, -0.75      , -0.59908691],
 [ 0.11960958, -0.91666667,  0.3813342 ]], dtype=float)


def _normalize(vectors):
    vectors = np.asarray(vectors, dtype=float)
    norms = np.linalg.norm(vectors, axis=-1, keepdims=True)
    norms = np.clip(norms, 1e-12, None)
    return vectors / norms


def _kernel(distances):
    scaled = np.clip(1.0 - 0.5 * np.asarray(distances, dtype=float), 0.0, None)
    return scaled**6


def support_function(nx, ny, nz, bias=DEFAULT_BIAS, weights=DEFAULT_WEIGHTS, centers=CENTERS):
    direction = _normalize(np.array([nx, ny, nz], dtype=float))
    distances = np.linalg.norm(centers - direction, axis=1)
    return float(bias + np.sum(weights * _kernel(distances)))


def surface_point(theta, phi, bias=DEFAULT_BIAS, weights=DEFAULT_WEIGHTS, centers=CENTERS):
    direction = np.array([
        np.sin(theta) * np.cos(phi),
        np.cos(theta),
        np.sin(theta) * np.sin(phi),
    ], dtype=float)
    direction = _normalize(direction)
    distances = np.linalg.norm(centers - direction, axis=1)
    h = float(bias + np.sum(weights * _kernel(distances)))
    gradient = np.zeros(3, dtype=float)
    for weight, center in zip(weights, centers):
        delta = direction - center
        distance = np.linalg.norm(delta)
        if distance < 1e-12:
            continue
        scaled = 1.0 - 0.5 * distance
        if scaled <= 0.0:
            continue
        derivative = -3.0 * scaled**5
        gradient += weight * derivative * delta / distance
    gradient -= np.dot(gradient, direction) * direction
    return h * direction + gradient
