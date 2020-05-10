#pragma once

/// \file
/// Utilities for rendering full images.

#include <thread>
#include <random>
#include <atomic>

#include "../math/vec.h"
#include "common.h"
#include "camera.h"

namespace fluid::renderer {
	/// Renders the scene to an image using an naive method for parallelization.
	template <bool Monitor = false, typename Incoming> image<spectrum> render_naive(
		Incoming &&li, const camera &cam, vec2s size, std::size_t spp, pcg32 &random
	) {
		using namespace std::chrono_literals;

		image<spectrum> result(size);
		std::uniform_real_distribution<double> dist(0.0, 1.0);
		vec2d screen_div = vec_ops::memberwise::div(vec2d(1.0, 1.0), vec2d(size));
		[[maybe_unused]] std::atomic<std::size_t> finished = 0;
		[[maybe_unused]] std::thread monitor_thread;
		if constexpr (Monitor) {
			monitor_thread = std::thread(
				[&finished](std::size_t total) {
					while (true) {
						std::size_t fin = finished;
						std::cout << fin << " / " << total << " (" << (100.0 * fin / static_cast<double>(total)) << "%)" << std::endl;
						if (fin == total) {
							break;
						}
						std::this_thread::sleep_for(100ms);
					}
				},
				size.x * size.y
					);
		}
#pragma omp parallel
		{
			pcg32 thread_rnd(random());
#pragma omp for
			for (int y = 0; y < size.y; ++y) {
				for (std::size_t x = 0; x < size.x; ++x) {
					spectrum res;
					for (std::size_t i = 0; i < spp; ++i) {
						vec2d pos = vec_ops::memberwise::mul(
							vec2d(vec2s(x, y)) + vec2d(dist(thread_rnd), dist(thread_rnd)), screen_div
						);
						res += li(cam.get_ray(pos), thread_rnd);
					}
					result.pixels(x, y) = res / static_cast<double>(spp);
					if constexpr (Monitor) {
						++finished;
					}
				}
			}
		}
		if constexpr (Monitor) {
			monitor_thread.join();
		}
		return result;
	}

	/// Accumulates incoming light to the given buffer using an naive method for parallelization.
	template <bool Monitor = false, typename Incoming> void accumulate_naive(
		Incoming &&li, image<spectrum> &buf, const camera &cam, std::size_t spp, pcg32 &random
	) {
		std::uniform_real_distribution<double> dist(0.0, 1.0);
		vec2d screen_div = vec_ops::memberwise::div(vec2d(1.0, 1.0), vec2d(buf.pixels.get_size()));
#pragma omp parallel
		{
			pcg32 thread_rnd(random());
#pragma omp for
			for (int y = 0; y < buf.pixels.get_size().y; ++y) {
				for (std::size_t x = 0; x < buf.pixels.get_size().x; ++x) {
					spectrum res;
					for (std::size_t i = 0; i < spp; ++i) {
						vec2d pos = vec_ops::memberwise::mul(
							vec2d(vec2s(x, y)) + vec2d(dist(thread_rnd), dist(thread_rnd)), screen_div
						);
						res += li(cam.get_ray(pos), thread_rnd);
					}
					buf.pixels(x, y) += res;
				}
			}
		}
	}

	/// Renders the scene to an image, dividing it into blocks.
	template <typename Incoming> image<spectrum> render_block(
		Incoming &&li, const camera &cam, vec2s size, std::size_t spp,
		pcg32 &random, vec2s block_size, std::size_t nthreads
	) {
		image<spectrum> result(size);
		vec2s num_blocks_dir = vec_ops::memberwise::div(size + block_size - vec2s(1, 1), block_size);
		std::size_t num_blocks = num_blocks_dir.x * num_blocks_dir.y;
		std::size_t blocks_per_thread = num_blocks / nthreads;
		// TODO
	}
}
