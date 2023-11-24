#ifndef _RANGE_UTILS_H_INCLUDED_
#define	_RANGE_UTILS_H_INCLUDED_

#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <climits>
#include <random>
#include <tuple>

namespace utils {
	static unsigned long x=123456789, y=362436069, z=521288629;

	unsigned long xorshf96(void) {          //period 2^96-1
		// return std::rand() / RAND_MAX;
		unsigned long t;
		x ^= x << 16;
		x ^= x >> 5;
		x ^= x << 1;

		t = x;
		x = y;
		y = z;
		z = t ^ x ^ y;

		return z;
	}
	float rgb2gray(float r, float g, float b) {
		return 0.229 * r + 0.587 * g + 0.114 * b;
	}
	int randrange(int min, int max) {
		return min + (rand() % (int)(max - min + 1));
	}
	std::vector<std::pair<int,int> > outline(int x, int y, bool use_corners) {
		std::vector<std::pair<int,int> > corners;

		corners.push_back(std::make_pair(x+1,y));
		corners.push_back(std::make_pair(x-1,y));
		corners.push_back(std::make_pair(x,y+1));
		corners.push_back(std::make_pair(x,y-1));

		if (use_corners) {
			corners.push_back(std::make_pair(x+1,y+1));
			corners.push_back(std::make_pair(x-1,y+1));
			corners.push_back(std::make_pair(x+1,y-1));
			corners.push_back(std::make_pair(x-1,y-1));
		}

		return corners;
	}
	template <class key_T>
	class KeyMaker
	{
	public:
		KeyMaker() {}
		KeyMaker(int width, int height, int theta_discretization) {
			y_shift = (int) std::ceil(std::log2(theta_discretization));
			x_shift = (int) std::ceil(std::log2(height)) + y_shift;
			int bitness = (int) std::ceil(std::log2(width)) + x_shift;

			if (bitness > std::log2(std::numeric_limits<key_T>::max())) {
				std::cerr << "Key bitness too large for integer packing scheme. Check your KeyMaker template type." << std::endl;
			}

			// make bit masks for unpacking the various values
			t_mask = std::pow(2, y_shift)-1;
			y_mask = std::pow(2, x_shift)-1 - t_mask;
			x_mask = std::pow(2, bitness)-1 - y_mask - t_mask;
		};
		~KeyMaker() {};
		key_T make_key(int x, int y, int t) {
			return ((key_T)x << x_shift) + ((key_T)y << y_shift) + (key_T)t;
		}
		std::tuple<int, int, int> unpack_key(key_T k) {
			return std::make_tuple((int)((k & x_mask) >> x_shift), (int)((k & y_mask) >> y_shift), k & t_mask);
		}
	private:
		int y_shift;
		int x_shift;
		key_T x_mask;
		key_T y_mask;
		key_T t_mask;
	};

	bool has(std::string substring, std::string str) {
		return str.find(substring) != std::string::npos;
	}

	bool has(std::string val, std::vector<std::string> vstr) {
		return std::find(vstr.begin(), vstr.end(),val)!=vstr.end();
	}

	std::vector<std::string> split(std::string in, char delim) {
		std::vector<std::string> result;
		std::stringstream ss(in);
		while( ss.good() )
		{
			std::string substr;
			std::getline( ss, substr, delim );
			result.push_back( substr );
		}
		return result;
	}

	double norminv(double q) {
		if(q == .5)
			return 0;

		q = 1.0 - q;

		double p = (q > 0.0 && q < 0.5) ? q : (1.0 - q);
		double t = sqrt(log(1.0 / pow(p, 2.0)));

		double c0 = 2.515517;
		double c1 = 0.802853;
		double c2 = 0.010328;

		double d1 = 1.432788;
		double d2 = 0.189269;
		double d3 = 0.001308;

		double x = t - (c0 + c1 * t + c2 * pow(t, 2.0)) /
					(1.0 + d1 * t + d2 * pow(t, 2.0) + d3 * pow(t, 3.0));

		if(q > .5)
		  x *= -1.0;

		return x;
	}

	template<typename T, typename U>
	struct is_same
	{
		static const bool value = false;
	};

	template<typename T>
	struct is_same<T, T>
	{
		static const bool value = true;
	};

	template<typename T, typename U>
	bool eqTypes() { return is_same<T, U>::value; }

	// http://stackoverflow.com/questions/311703/algorithm-for-sampling-without-replacement
	// Here's some code for sampling without replacement based on Algorithm 3.4.2S of Knuth's book Seminumeric Algorithms.
	class NonReplacementSampler
	{
	public:
		NonReplacementSampler() {
			rand = std::uniform_real_distribution<double>(0.0,1.0);
			generator.seed(clock());
		}
		~NonReplacementSampler() {}

		void sample(int populationSize, int sampleSize, std::vector<int> & samples) {
			int t = 0; // total input records dealt with
			int m = 0; // number of items selected so far
			double u;

			while (m < sampleSize) {
				// u = rand(generator);
				u = std::rand() / (float)RAND_MAX;
				if ((populationSize-t)*u >= sampleSize - m) t++;
				else {
					samples.push_back(t);
					t++; m++;
				}
			}
			// samples.push_back(1);
		}

		std::uniform_real_distribution<double> rand;
		std::default_random_engine generator;
	};

	class FastRand
	{
	public:
		FastRand() : FastRand(10000) {};
		FastRand(int n) : cache_size(n) {
			populate();
			repopulate_threshold = 1.0 / cache_size;
		}
		~FastRand(){};

		float rand() {
			// return std::rand() / (float)RAND_MAX;
			// float v = cache[i];
			if (i++>cache_size-1) i = 0;
			// if (v < repopulate_threshold) populate();
			return cache[i];
		}

		void populate() {
			// cache.empty();
			// for (int i = 0; i < cache_size; ++i) cache.push_back(std::rand() / (float)RAND_MAX);
			for (int i = 0; i < cache_size; ++i) cache[i] = std::rand() / (float)RAND_MAX;
		}
		int i = 0;
		int cache_size;
		float repopulate_threshold;
		// std::vector<float> cache;
		float cache[10000];
	};

	void serialize(std::vector<bool> &vals,std::stringstream* ss) {
		if (vals.size() == 0) {
			(*ss) << "[]";
			return; 
		}
		(*ss) << "[" << vals[0];
		for (int i = 1; i < vals.size(); ++i) {
			(*ss) << "," << vals[i];
		}
		(*ss) << "]";
	}

	void serialize(std::vector<float> &vals,std::stringstream* ss) {
		if (vals.size() == 0) {
			(*ss) << "[]";
			return; 
		}
		(*ss) << "[" << vals[0];
		for (int i = 1; i < vals.size(); ++i) {
			(*ss) << "," << vals[i];
		}
		(*ss) << "]";
	}

	std::string serialize(std::vector<float> &vals) {
		std::stringstream ss;
		serialize(vals,&ss);
		return ss.str();
	}

} // namespace utils

#endif	/* _RANGE_UTILS_H_INCLUDED_ */
