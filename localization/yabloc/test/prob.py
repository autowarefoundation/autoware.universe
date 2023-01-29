#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np


def normal_distribution(x, mu, sigma):
    return 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (x - mu)**2 / (2 * sigma**2))


class PriorDistribution:
    def __init__(self, mu=0, sigma=1.0):
        self.mu_ = mu
        self.sigma_ = sigma

    def prob(self, x):
        return normal_distribution(x, self.mu_, self.sigma_)

    def sample(self, n=100):
        return np.random.normal(self.mu_, self.sigma_, n)

    def set_sigma(self, sigma):
        self.sigma_ = sigma


class CameraDistribution:
    def prob(self, x):
        # NOTE: This is not probability distribution
        # Because the total is not 1
        mod = (x + 1) % 3
        if mod < 1:
            return 0.5
        return 0.0


dist1 = PriorDistribution()
dist2 = CameraDistribution()


def draw_theoretical_distribution(ax, x):
    # Plot well known distributions
    ax.plot(x, np.vectorize(dist1.prob)(x),
            label='prior PDF(normal)', color='b')
    ax.plot(x, np.vectorize(dist2.prob)(x), label='measurement', color='r')

    # Plot posterior distribution
    def multi(x):
        return dist1.prob(x) * dist2.prob(x)
    y = np.vectorize(multi)(x)
    dx = x[1] - x[0]  # $\sum normalized_y \cdot dx$ must be $1$
    normalized_y = y / np.sum(y) / dx
    ax.plot(x, normalized_y, label='posterior PDF', color='g')

    # Plot approximated posterior distribution
    mu = np.sum(normalized_y * x) / np.sum(normalized_y)
    var = np.sum(normalized_y * (x - mu)**2) / np.sum(normalized_y)
    sigma = np.sqrt(var)
    ax.plot(x, normal_distribution(x, mu, sigma),
            label='posterior PDF(normal approx)', color='g', linestyle='dashed')

    # NOTE: This is meaning-less
    # # Plot approximated measurement distribution
    # inv_gains = 1. / np.vectorize(dist1.prob)(x)
    # mu = np.sum(normalized_y * x * inv_gains) / \
    #     np.sum(normalized_y * inv_gains)
    # var = np.sum(normalized_y * inv_gains * (x - mu)**2) / \
    #     np.sum(normalized_y * inv_gains)
    # sigma = np.sqrt(var)
    # print('theoretical: mean=', mu, 'std=', sigma)
    # ax.plot(x, normal_distribution(x, mu, sigma),
    #         label='measurement PDF(normal approx)', color='r', linestyle='dashed')

    dist3 = PriorDistribution(mu=mu, sigma=sigma)
    normalized_y = np.vectorize(dist3.prob)(x)
    inv_gains = 1. / np.vectorize(dist1.prob)(x)
    mu2 = np.sum(normalized_y * x * inv_gains) / \
        np.sum(normalized_y * inv_gains)
    var2 = np.sum(normalized_y * inv_gains * (x - mu2)**2) / \
        np.sum(normalized_y * inv_gains)
    sigma2 = np.sqrt(var2)
    ax.plot(x, normal_distribution(x, mu2, sigma2),
            label='measurement PDF(normal approx)', color='r', linestyle='dashed')

    # DEBUG: This prove the above computation is right
    # Simulate kalman filter
    # mu = dist1.mu_
    # sigma = dist1.sigma_
    # K = sigma**2 / (sigma**2 + sigma2**2)
    # mu3 = mu + K * (mu2 - mu)
    # var3 = (1 - K) * sigma**2
    # sigma3 = np.sqrt(var3)
    # ax.plot(x, normal_distribution(x, mu3, sigma3),
    #         label='debug', color='g', linewidth=3, linestyle='dotted')


def draw_particle_distribution(ax, x):
    N = 10000
    prior_particles = dist1.sample(N)
    weights = np.vectorize(dist2.prob)(prior_particles)
    weighted_partilces = weights * prior_particles

    count, bins, ignored = ax.hist(
        prior_particles, 50, density=True, alpha=0.2, color='b')

    def resample():
        resampled = np.zeros(N)
        thresh = 1. / N
        sum_w = 0
        next_id = 0
        normalize = np.sum(weights)
        for (p, w) in zip(prior_particles, weights):
            sum_w += w / normalize
            while sum_w > thresh:
                resampled[next_id] = p
                next_id += 1
                thresh = (next_id + 1) / N
                if next_id == N:
                    break
            if next_id == N:
                break
        return resampled

    resampled = resample()
    count, bins, ignored = ax.hist(
        resampled, 30, density=True, alpha=0.2, color='g')

    # Conventional computation
    mu = np.sum(weighted_partilces) / np.sum(weights)
    tmp = weights * (prior_particles - mu) ** 2
    var = np.sum(tmp) / np.sum(weights)
    sigma = np.sqrt(var)
    ax.plot(x, normal_distribution(x, mu, sigma),
            label='Monte Carlo: posterior PDF(normal approx)', color='g', linestyle='dotted')

    # Proposed computation
    def proposed():
        tmp = np.vectorize(dist1.prob)(prior_particles)
        tmp[tmp < 1e-3] = 1e-3
        inv_gains = 1. / tmp

        regularized_particles = inv_gains * weighted_partilces
        mu = np.sum(regularized_particles) / np.sum(weights * inv_gains)
        tmp = inv_gains * weights * (prior_particles - mu) ** 2
        var = np.sum(tmp) / np.sum(weights * inv_gains)
        sigma = np.sqrt(var)
        print('monte carlo mean=', mu, "std=", sigma)
        ax.plot(x, normal_distribution(x, mu, sigma),
                label='Monte Carlo: measurement PDF(normal approx)', color='r', linestyle='dotted')

    def new_proposed():
        tmp = np.vectorize(dist1.prob)(resampled)
        tmp[tmp < 1e-3] = 1e-3
        weights = 1. / tmp

        def reresample():
            reresampled = np.zeros(N)
            thresh = 1. / N
            sum_w = 0
            next_id = 0
            normalize = np.sum(weights)
            for (p, w) in zip(resampled, weights):
                sum_w += w / normalize
                while sum_w > thresh:
                    reresampled[next_id] = p
                    next_id += 1
                    thresh = (next_id + 1) / N
                    if next_id == N:
                        break
                if next_id == N:
                    break
            return reresampled

        reresampled_particles = reresample()
        count, bins, ignored = ax.hist(
            reresampled_particles, 30, density=True, alpha=0.2, color='r')

        mu = np.mean(reresampled_particles)
        sigma = np.std(reresampled_particles)
        ax.plot(x, normal_distribution(x, mu, sigma),
                label='Monte Carlo: measurement PDF(normal approx)', color='r', linestyle='dotted')
    new_proposed()


def main():
    resolution = np.linspace(-5., 5., 101)
    fig = plt.figure(figsize=(15, 6))
    ax = fig.subplots(1, 2)

    ax[0].grid(True)
    ax[1].grid(True)
    draw_theoretical_distribution(ax[0], resolution)
    draw_particle_distribution(ax[1], resolution)

    for a in ax:
        a.set_title('multi lane situation')
        a.set_xlim([-5, 5])
        a.set_ylim([-0.1, 1.3])
        a.legend(loc='upper right')
    plt.pause(1.0)
    plt.show()


if __name__ == '__main__':
    main()
