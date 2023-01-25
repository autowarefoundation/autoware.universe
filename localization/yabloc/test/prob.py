#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np


def normal_distribution(x, mu, sigma):
    return 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (x - mu)**2 / (2 * sigma**2))


class PriorDistribution:
    def __init__(self):
        self.mu_ = 0
        self.sigma_ = 1

    def prob(self, x):
        return normal_distribution(x, self.mu_, self.sigma_)

    def sample(self, n=100):
        return np.random.normal(self.mu_, self.sigma_, n)


class CameraDistribution:
    def prob(self, x):
        # NOTE: This is not probability distribution
        # Because the summation is not 1
        mod = (x + 1) % 3
        if mod < 1:
            return 0.5
        return 0.0


dist1 = PriorDistribution()
dist2 = CameraDistribution()


def draw_theoretical_distribution(ax, x):
    # plot well known distributions
    ax.plot(x, np.vectorize(dist1.prob)(x),
            label='prior PDF(normal)', color='b')
    ax.plot(x, np.vectorize(dist2.prob)(x), label='measurement', color='r')

    # plot posterior distribution
    def multi(x):
        return dist1.prob(x) * dist2.prob(x)
    y = np.vectorize(multi)(x)
    dx = x[1] - x[0]  # $\sum normalized_y \cdot dx$ must be $1$
    normalized_y = y / np.sum(y) / dx
    ax.plot(x, normalized_y, label='posterior PDF', color='g')

    # plot approximated posterior distribution
    ax.plot(x, normal_distribution(x, np.mean(normalized_y), np.std(
        normalized_y)), label='posterior PDF(normal approx)', color='g', linestyle='dashed')

    # plot approximated measurement distribution
    # regen_measurement = y / np.vectorize(dist1.prob)(x)
    # print(regen_measurement)
    # ax.plot(x, regen_measurement, label='measurement',
    #         color='r', linestyle='dashed', linewidth=3)


def draw_particle_distribution(ax, x):
    N = 5000
    prior_particles = dist1.sample(N)
    weights = np.vectorize(dist2.prob)(prior_particles)
    weighted_partilces = weights * prior_particles

    # DEBUG:
    # ax.scatter(prior_particles, weights)
    # count, bins, ignored = plt.hist(
    #     weighted_partilces, 30, density=True)

    mu = np.sum(weighted_partilces) / np.sum(weights)
    if True:  # use weights for variance computation
        tmp = weights * (prior_particles - mu) ** 2
        var = np.sum(tmp) / np.sum(weights)
    else:
        tmp = (prior_particles - mu) ** 2
        var = np.sum(tmp) / N
    sigma = np.sqrt(var)

    print('mu', mu)
    print('sigma', sigma)
    ax.plot(x, normal_distribution(x, mu, sigma),
            label='posterior P-PDF(normal approx)', color='g', linestyle='dotted')


def main():
    resolution = np.linspace(-5., 5., 101)
    fig = plt.figure(figsize=(15, 6))
    ax = fig.subplots()
    ax.grid(True)

    draw_theoretical_distribution(ax, resolution)
    draw_particle_distribution(ax, resolution)
    ax.set_title('multi lane situation')
    ax.legend()
    plt.show()


if __name__ == '__main__':
    main()
