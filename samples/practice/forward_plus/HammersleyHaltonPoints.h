#pragma once
#include "common/glm_common.h"

// Hammersley points on 2d plane with p1 = 2
void Hammersley(float *result, int n) {
    float p, u, v;
    int   k, kk, pos;
    for (k = 0, pos = 0; k < n; k++) {
        u = 0;
        for (p = 0.5, kk = k; kk; p *= 0.5, kk >>= 1) {
            if (kk & 1) {
                // kk mod 2 == 1
                u += p;
            }
        }

        v = (k + 0.5f) / n;
        result[pos++] = u;
        result[pos++] = v;
    }
}

// Halton point on 2d plane with p1 = 2
void Halton(float *result, int n, int p2) {
    float p, u, v, ip;
    int   k, kk, pos, a;
    for (k = 0, pos = 0; k < n; k++) {
        u = 0;
        for (p = 0.5, kk = k; kk; p *= 0.5, kk >>= 1) {
            if (kk & 1) {
                // kk mod 2 == 1
                u += p;
            }
        }

        v = 0;
        ip = 1.0f / p2;
        // inverse of p2
        for (p = ip, kk = k; kk; p *= ip, kk /= p2) {
            // kk = (int)(kk/p2)
            if ((a = kk % p2)) {
                v += a * p;
            }
        }

        result[pos++] = u;
        result[pos++] = v;
    }
}

// Hammersley point on sphere with p1 = 2
void SphereHammersley(float *result, int n) {
    float p, t, st, phi, phirad;
    int   k, kk, pos;
    for (k = 0, pos = 0; k < n; k++) {
        t = 0;
        for (p = 0.5, kk = k; kk; p *= 0.5, kk >>= 1) {
            if (kk & 1) {
                // kk mod 2 == 1
                t += p;
            }
        }

        t = 2.0f * t - 1.0f;
        // map from [0,1] to [-1,1]
        phi = (k + 0.5f) / n;
        // a slight shift
        phirad = phi * glm::two_pi<float>();
        // map to [0, 2 pi)
        st = glm::sqrt(1.0f - t * t);

        result[pos++] = st * glm::cos(phirad);
        result[pos++] = st * glm::sin(phirad);
        result[pos++] = t;
    }
}

// Halton point on sphere with p1 = 2
void SphereHalton(float *result, int n, int p2) {
    float p, t, st, phi, phirad, ip;
    int   k, kk, pos, a;
    for (k = 0, pos = 0; k < n; k++) {
        t = 0;
        for (p = 0.5, kk = k; kk; p *= 0.5, kk >>= 1) {
            if (kk & 1) {
                // kk mod 2 == 1
                t += p;
            }
        }

        t = 2.0f * t - 1.0f;
        // map from [0,1] to [-1,1]
        st = sqrt(1.0f - t * t);
        phi = 0.0f;
        ip = 1.0f / p2;

        for (p = ip, kk = k; kk; p *= ip, kk /= p2) {
            // kk = (int)(kk/p2)
            if ((a = kk % p2)) {
                phi += a * p;
            }
        }
        phirad = phi * 4.0f * glm::pi<float>();
        // map from [0,0.5] to [0, 2 pi)
        result[pos++] = st * glm::cos(phirad);
        result[pos++] = st * glm::sin(phirad);
        result[pos++] = t;
    }
}

void GenerateHimesphereHammersleyPoints(float* result, int32_t n) {
    const float pi = 3.1415926f;
    float p, t, st, phi, phirad, phirady;
    int32_t k, kk, pos;
    for (k = 0, pos = 0; k < n; k++) {
        t = 0;
        for (p = 0.5f, kk = k; kk; p *= 0.5f, kk >>= 1)
            if (kk & 1)		// kk mod 2 == 1
                t += p;
        t = 1.0 - t;
        phi = (k + 0.5f) / n;	// a slight shift
        phirad = phi * 2.0f * pi;	// map to [0, 2 pi)
        st = sqrt(1.0f - t * t);
        glm::vec3 p = glm::normalize(glm::vec3(st * glm::cos(phirad), st * glm::sin(phirad), t));

        result[pos * 3 + 0] = p.x;
        result[pos * 3 + 1] = p.y;
        result[pos * 3 + 2] = p.z;
        ++pos;
    }
}

void GenerateHimesphereHammersleyPointsWithCos(float * result, int32_t n) {
    const float pi = 3.1415926f;
    float p, t, st, phi, phirad, phirady;
    int32_t k, kk, pos;
    for (k = 0, pos = 0; k < n; k++) {
        t = 0;
        for (p = 0.5f, kk = k; kk; p *= 0.5f, kk >>= 1)
            if (kk & 1)		// kk mod 2 == 1
                t += p;
        t = sqrt(1.0 - t);
        phi = (k + 0.5f) / n;	// a slight shift
        phirad = phi * 2.0f * pi;	// map to [0, 2 pi)
        st = sqrt(1.0f - t * t);
        glm::vec3 p = glm::normalize(glm::vec3(st * glm::cos(phirad), st * glm::sin(phirad), t));

        result[pos * 3 + 0] = p.x;
        result[pos * 3 + 1] = p.y;
        result[pos * 3 + 2] = p.z;
        ++pos;
    }
}