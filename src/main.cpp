#include <iostream>

#include "math/vec.h"

using namespace fluid;

int main() {
	vec2f v1(1.0f, 2.0f), v2(-3.0f, 5.0f);
	std::cout << v1.squared_length() << "\n";
	vec2f v3 = (v1 + v2) / 4.0f;
	vec2f v4 = v3.normalized_checked().value_or(vec2f(1.0f, 0.0f));
	vec2i vi(1, 2);

	vec3f v31 = vec_ops::cross(vec3f(1.0f, 2.0f, 3.0f), vec3f(-5.0f, -3.0f, 4.0f));
	std::cout << v31.length() << "\n";

	return 0;
}
