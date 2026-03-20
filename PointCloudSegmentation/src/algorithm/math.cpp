#include <src/algorithm/math.h>

namespace d3s {
	namespace pcs {

		double barycentricInterpolation(double x1,
										double y1,
										double z1,
										double x2,
										double y2,
										double z2,
										double x3,
										double y3,
										double z3,
										double x,
										double y)
		{
			double z = std::numeric_limits<double>::infinity();

			double detT = ((y2 - y3) * (x1 - x3)) + ((x3 - x2) * (y1 - y3));

			// ABELL - should probably check something close to 0, rather than
			// exactly 0.
			if (detT != 0.0)
			{
				// Compute the barycentric coordinates of x,y (relative to
				// x1/y1, x2/y2, x3/y3).  Essentially the weight that each
				// corner of the triangle contributes to the point in question.

				// Another way to think about this is that we're making a basis
				// for the system with the basis vectors being two sides of
				// the triangle.  You can rearrange the z calculation below in
				// terms of lambda1 and lambda2 to see this.  Also note that
				// since lambda1 and lambda2 are coefficients of the basis vectors,
				// any values outside of the range [0,1] are necessarily out of the
				// triangle.
				double lambda1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / detT;
				double lambda2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / detT;

				if (lambda1 >= 0 && lambda1 <= 1 && lambda2 >= 0 && lambda2 <= 1)
				{
					double sum = lambda1 + lambda2;
					if (sum <= 1)
					{
						double lambda3 = 1 - sum;
						z = (lambda1 * z1) + (lambda2 * z2) + (lambda3 * z3);
					}
				}
			}

			return z;
		}

	}
}