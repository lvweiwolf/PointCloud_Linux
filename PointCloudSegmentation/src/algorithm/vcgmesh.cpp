#include <src/algorithm/vcgmesh.h>
#include <src/utils/logging.h>

#include <vcg/math/quadric.h>
#include <vcg/complex/algorithms/edge_collapse.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>

using namespace vcg;

namespace d3s {
	namespace pcs {


		namespace {
			typedef SimpleTempData<CMeshO::VertContainer, math::Quadric<double>> QuadricTemp;
			typedef vcg::tri::BasicVertexPair<CVertexO> VertexPair;


			template <typename DerivedP,
					  typename DerivedA,
					  typename DerivedB,
					  typename DerivedC,
					  typename DerivedL>
			void barycentric_coordinates(const Eigen::MatrixBase<DerivedP>& P,
										 const Eigen::MatrixBase<DerivedA>& A,
										 const Eigen::MatrixBase<DerivedB>& B,
										 const Eigen::MatrixBase<DerivedC>& C,
										 Eigen::PlainObjectBase<DerivedL>& L)
			{
				using namespace Eigen;
#ifndef NDEBUG
				const int DIM = P.cols();
				CHECK(A.cols() == DIM && "corners must be in same dimension as query");
				CHECK(B.cols() == DIM && "corners must be in same dimension as query");
				CHECK(C.cols() == DIM && "corners must be in same dimension as query");
				CHECK(P.rows() == A.rows() && "Must have same number of queries as corners");
				CHECK(A.rows() == B.rows() && "Corners must be same size");
				CHECK(A.rows() == C.rows() && "Corners must be same size");
#endif

				// http://gamedev.stackexchange.com/a/23745
				typedef Eigen::Array<typename DerivedP::Scalar,
									 DerivedP::RowsAtCompileTime,
									 DerivedP::ColsAtCompileTime>
					ArrayS;
				typedef Eigen::Array<typename DerivedP::Scalar, DerivedP::RowsAtCompileTime, 1>
					VectorS;

				const ArrayS v0 = B.array() - A.array();
				const ArrayS v1 = C.array() - A.array();
				const ArrayS v2 = P.array() - A.array();
				VectorS d00 = (v0 * v0).rowwise().sum();
				VectorS d01 = (v0 * v1).rowwise().sum();
				VectorS d11 = (v1 * v1).rowwise().sum();
				VectorS d20 = (v2 * v0).rowwise().sum();
				VectorS d21 = (v2 * v1).rowwise().sum();
				VectorS denom = d00 * d11 - d01 * d01;
				L.resize(P.rows(), 3);
				L.col(1) = (d11 * d20 - d01 * d21) / denom;
				L.col(2) = (d00 * d21 - d01 * d20) / denom;
				L.col(0) = 1.0f - (L.col(1) + L.col(2)).array();
			}

			template <typename Real>
			void barycentric_coordinates(const vcg::Point3<Real>& p,
										 const vcg::Point3<Real>& a,
										 const vcg::Point3<Real>& b,
										 const vcg::Point3<Real>& c,
										 vcg::Point3<Real>& l)
			{
				Eigen::MatrixXd P;
				P.resize(1, 3);
				P << p[0], p[1], p[2];
				Eigen::MatrixXd A;
				A.resize(1, 3);
				A << a[0], a[1], a[2];
				Eigen::MatrixXd B;
				B.resize(1, 3);
				B << b[0], b[1], b[2];
				Eigen::MatrixXd C;
				C.resize(1, 3);
				C << c[0], c[1], c[2];

				Eigen::MatrixXd L;

				barycentric_coordinates(P, A, B, C, L);

				l.FromEigenVector(L.row(0));
			}


			template <typename Real, typename Type0, typename Type1>
			class DCPQuery
			{
			public:
				struct Result
				{
				};

				Result operator()(Type0 const& primitive0, Type1 const& primitive1);
			};

			template <typename Real>
			class DCPQuery<Real, vcg::Point3<Real>, vcg::Plane3<Real>>
			{
			public:
				struct Result
				{
					Real distance, signedDistance;
					vcg::Point3<Real> planeClosestPoint;
				};

				Result operator()(vcg::Point3<Real> const& point, vcg::Plane3<Real> const& plane)
				{
					Result result;
					result.signedDistance = plane.Direction().dot(point) - plane.Offset();
					result.distance = std::fabs(result.signedDistance);
					result.planeClosestPoint = point - plane.Direction() * result.signedDistance;
					return result;
				}
			};


			class SplitMeshByPlane
			{
			public:
				typedef Scalarm Real;

				struct Face
				{
					int v[3];
					vcg::TexCoord2<float> tex[3];
				};

			public:
				typedef std::pair<int, int> EdgeKey;
				typedef std::vector<vcg::Point3<Real>> VertexArray;
				typedef std::vector<Face> FaceArray;

				VertexArray clip_vertices;
				std::vector<Real> signed_distances;
				std::map<EdgeKey, std::pair<Point3m, int>> edges_map;

				// 被切分的两个网格
				CMeshO negative_mesh, positive_mesh;

				void operator()(CMeshO& mesh, const vcg::Plane3<Real>& plane)
				{
					edges_map.clear();
					signed_distances.resize(mesh.VN());

					for (auto vi = mesh.vert.begin(); vi != mesh.vert.end(); ++vi)
						clip_vertices.push_back(vi->cP());

					ClassifyVertices(clip_vertices, plane);
					ClassifyEdges(mesh, clip_vertices);

					FaceArray negative, positive;
					ClassifyTriangles(mesh, negative, positive);

					UpdateDataMask(negative_mesh, mesh.currentDataMask);
					CreateMesh(negative, negative_mesh);

					UpdateDataMask(positive_mesh, mesh.currentDataMask);
					CreateMesh(positive, positive_mesh);
				}

			private:
				void ClassifyVertices(const VertexArray& clipVertices,
									  const vcg::Plane3<Real>& plane)
				{
					DCPQuery<Real, vcg::Point3<Real>, vcg::Plane3<Real>> query;

					for (size_t i = 0; i < clipVertices.size(); ++i)
						signed_distances[i] = query(clipVertices[i], plane).signedDistance;
				}

				void ClassifyEdges(const CMeshO& mesh, VertexArray& clipVertices)
				{
					int nextIndex = static_cast<int>(clipVertices.size());

					for (auto fi = mesh.face.begin(); fi != mesh.face.end(); ++fi)
					{
						int v0 = fi->cV(0)->Index();
						int v1 = fi->cV(1)->Index();
						int v2 = fi->cV(2)->Index();

						Real dist0 = signed_distances[v0];
						Real dist1 = signed_distances[v1];
						Real dist2 = signed_distances[v2];

						EdgeKey key;
						Real t;
						vcg::Point3<Real> intr, diff;

						if ((dist0 > (Real)0 && dist1 < (Real)0) ||
							(dist0 < (Real)0 && dist1 > (Real)0))
						{
							key = EdgeKey(v0, v1);

							if (edges_map.find(key) == edges_map.end())
							{
								t = dist0 / (dist0 - dist1);

								diff = clipVertices[v1] - clipVertices[v0];
								intr = clipVertices[v0] + diff * t;

								clipVertices.push_back(intr);
								edges_map[key] = std::make_pair(intr, nextIndex);
								++nextIndex;
							}
						}


						if ((dist1 > (Real)0 && dist2 < (Real)0) ||
							(dist1 < (Real)0 && dist2 > (Real)0))
						{
							key = EdgeKey(v1, v2);

							if (edges_map.find(key) == edges_map.end())
							{
								t = dist1 / (dist1 - dist2);
								diff = clipVertices[v2] - clipVertices[v1];
								intr = clipVertices[v1] + diff * t;

								clipVertices.push_back(intr);
								edges_map[key] = std::make_pair(intr, nextIndex);
								++nextIndex;
							}
						}

						if ((dist2 > (Real)0 && dist0 < (Real)0) ||
							(dist2 < (Real)0 && dist0 > (Real)0))
						{
							key = EdgeKey(v2, v0);

							if (edges_map.find(key) == edges_map.end())
							{
								t = dist2 / (dist2 - dist0);
								diff = clipVertices[v0] - clipVertices[v2];
								intr = clipVertices[v2] + diff * t;

								clipVertices.push_back(intr);
								edges_map[key] = std::make_pair(intr, nextIndex);
								++nextIndex;
							}
						}
					}
				}

				void ClassifyTriangles(const CMeshO& mesh, FaceArray& negative, FaceArray& positive)
				{
					for (auto fi = mesh.face.begin(); fi != mesh.face.end(); ++fi)
					{
						int v0 = fi->cV(0)->Index();
						int v1 = fi->cV(1)->Index();
						int v2 = fi->cV(2)->Index();

						auto t0 = fi->cWT(0);
						auto t1 = fi->cWT(1);
						auto t2 = fi->cWT(2);

						Real dist0 = signed_distances[v0];
						Real dist1 = signed_distances[v1];
						Real dist2 = signed_distances[v2];

						if (dist0 > (Real)0)
						{
							if (dist1 > (Real)0)
							{
								if (dist2 > (Real)0)
								{
									// +++
									AppendTriangle(positive, v0, v1, v2, t0, t1, t2);
								}
								else if (dist2 < (Real)0)
								{
									// ++-
									SplitTrianglePPM(negative, positive, v0, v1, v2, t0, t1, t2);
								}
								else
								{
									// ++0
									AppendTriangle(positive, v0, v1, v2, t0, t1, t2);
								}
							}
							else if (dist1 < (Real)0)
							{
								if (dist2 > (Real)0)
								{
									// +-+
									SplitTrianglePPM(negative, positive, v2, v0, v1, t2, t0, t1);
								}
								else if (dist2 < (Real)0)
								{
									// +--
									SplitTriangleMMP(negative, positive, v1, v2, v0, t1, t2, t0);
								}
								else
								{
									// +-0
									SplitTrianglePMZ(negative, positive, v0, v1, v2, t0, t1, t2);
								}
							}
							else
							{
								if (dist2 > (Real)0)
								{
									// +0+
									AppendTriangle(positive, v0, v1, v2, t0, t1, t2);
								}
								else if (dist2 < (Real)0)
								{
									// +0-
									SplitTriangleMPZ(negative, positive, v2, v0, v1, t2, t0, t1);
								}
								else
								{
									// +00
									AppendTriangle(positive, v0, v1, v2, t0, t1, t2);
								}
							}
						}
						else if (dist0 < (Real)0)
						{
							if (dist1 > (Real)0)
							{
								if (dist2 > (Real)0)
								{
									// -++
									SplitTrianglePPM(negative, positive, v1, v2, v0, t1, t2, t0);
								}
								else if (dist2 < (Real)0)
								{
									// -+-
									SplitTriangleMMP(negative, positive, v2, v0, v1, t2, t0, t1);
								}
								else
								{
									// -+0
									SplitTriangleMPZ(negative, positive, v0, v1, v2, t0, t1, t2);
								}
							}
							else if (dist1 < (Real)0)
							{
								if (dist2 > (Real)0)
								{
									// --+
									SplitTriangleMMP(negative, positive, v0, v1, v2, t0, t1, t2);
								}
								else if (dist2 < (Real)0)
								{
									// ---
									AppendTriangle(negative, v0, v1, v2, t0, t1, t2);
								}
								else
								{
									// --0
									AppendTriangle(negative, v0, v1, v2, t0, t1, t2);
								}
							}
							else
							{
								if (dist2 > (Real)0)
								{
									// -0+
									SplitTrianglePMZ(negative, positive, v2, v0, v1, t2, t0, t1);
								}
								else if (dist2 < (Real)0)
								{
									// -0-
									AppendTriangle(negative, v0, v1, v2, t0, t1, t2);
								}
								else
								{
									// -00
									AppendTriangle(negative, v0, v1, v2, t0, t1, t2);
								}
							}
						}
						else
						{
							if (dist1 > (Real)0)
							{
								if (dist2 > (Real)0)
								{
									// 0++
									AppendTriangle(positive, v0, v1, v2, t0, t1, t2);
								}
								else if (dist2 < (Real)0)
								{
									// 0+-
									SplitTrianglePMZ(negative, positive, v1, v2, v0, t1, t2, t0);
								}
								else
								{
									// 0+0
									AppendTriangle(positive, v0, v1, v2, t0, t1, t2);
								}
							}
							else if (dist1 < (Real)0)
							{
								if (dist2 > (Real)0)
								{
									// 0-+
									SplitTriangleMPZ(negative, positive, v1, v2, v0, t1, t2, t0);
								}
								else if (dist2 < (Real)0)
								{
									// 0--
									AppendTriangle(negative, v0, v1, v2, t0, t1, t2);
								}
								else
								{
									// 0-0
									AppendTriangle(negative, v0, v1, v2, t0, t1, t2);
								}
							}
							else
							{
								if (dist2 > (Real)0)
								{
									// 00+
									AppendTriangle(positive, v0, v1, v2, t0, t1, t2);
								}
								else if (dist2 < (Real)0)
								{
									// 00-
									AppendTriangle(negative, v0, v1, v2, t0, t1, t2);
								}
								else
								{
									// 000, reject triangles lying in the plane
								}
							}
						}
					}
				}

				void AppendTriangle(FaceArray& faces,
									int v0,
									int v1,
									int v2,
									vcg::TexCoord2<float>& t0,
									vcg::TexCoord2<float>& t1,
									vcg::TexCoord2<float>& t2)
				{
					Face face = { { v0, v1, v2 }, { t0, t1, t2 } };
					faces.push_back(face);
				}

				void SplitTrianglePPM(FaceArray& negative,
									  FaceArray& positive,
									  int v0,
									  int v1,
									  int v2,
									  vcg::TexCoord2<float>& t0,
									  vcg::TexCoord2<float>& t1,
									  vcg::TexCoord2<float>& t2)
				{
					CHECK(t0.n() == t1.n() && t1.n() == t2.n());

					int v12 = edges_map[EdgeKey(v1, v2)].second;
					int v20 = edges_map[EdgeKey(v2, v0)].second;

					const auto& p0 = clip_vertices[v0];
					const auto& p1 = clip_vertices[v1];
					const auto& p2 = clip_vertices[v2];
					const auto& p12 = clip_vertices[v12];
					const auto& p20 = clip_vertices[v20];

					vcg::Point3<Real> bary_coord_p12, bary_coord_p20;
					barycentric_coordinates(p12, p0, p1, p2, bary_coord_p12);
					barycentric_coordinates(p20, p0, p1, p2, bary_coord_p20);

					vcg::TexCoord2<float> t12;
					t12.u() = t0.u() * bary_coord_p12[0] + t1.u() * bary_coord_p12[1] +
							  t2.u() * bary_coord_p12[2];
					t12.v() = t0.v() * bary_coord_p12[0] + t1.v() * bary_coord_p12[1] +
							  t2.v() * bary_coord_p12[2];
					t12.n() = t0.n();

					vcg::TexCoord2<float> t20;
					t20.u() = t0.u() * bary_coord_p20[0] + t1.u() * bary_coord_p20[1] +
							  t2.u() * bary_coord_p20[2];
					t20.v() = t0.v() * bary_coord_p20[0] + t1.v() * bary_coord_p20[1] +
							  t2.v() * bary_coord_p20[2];
					t20.n() = t0.n();

					Face pos_face_1 = { { v0, v1, v12 }, { t0, t1, t12 } };
					positive.push_back(pos_face_1);

					Face pos_face_2 = { { v0, v12, v20 }, { t0, t12, t20 } };
					positive.push_back(pos_face_2);

					Face neg_face_1 = { { v2, v20, v12 }, { t2, t20, t12 } };
					negative.push_back(neg_face_1);
				}


				void SplitTriangleMMP(FaceArray& negative,
									  FaceArray& positive,
									  int v0,
									  int v1,
									  int v2,
									  vcg::TexCoord2<float>& t0,
									  vcg::TexCoord2<float>& t1,
									  vcg::TexCoord2<float>& t2)
				{
					CHECK(t0.n() == t1.n() && t1.n() == t2.n());
					int v12 = edges_map[EdgeKey(v1, v2)].second;
					int v20 = edges_map[EdgeKey(v2, v0)].second;

					const auto& p0 = clip_vertices[v0];
					const auto& p1 = clip_vertices[v1];
					const auto& p2 = clip_vertices[v2];
					const auto& p12 = clip_vertices[v12];
					const auto& p20 = clip_vertices[v20];

					vcg::Point3<Real> bary_coord_p12, bary_coord_p20;
					barycentric_coordinates(p12, p0, p1, p2, bary_coord_p12);
					barycentric_coordinates(p20, p0, p1, p2, bary_coord_p20);

					vcg::TexCoord2<float> t12;
					t12.u() = t0.u() * bary_coord_p12[0] + t1.u() * bary_coord_p12[1] +
							  t2.u() * bary_coord_p12[2];
					t12.v() = t0.v() * bary_coord_p12[0] + t1.v() * bary_coord_p12[1] +
							  t2.v() * bary_coord_p12[2];
					t12.n() = t0.n();

					vcg::TexCoord2<float> t20;
					t20.u() = t0.u() * bary_coord_p20[0] + t1.u() * bary_coord_p20[1] +
							  t2.u() * bary_coord_p20[2];
					t20.v() = t0.v() * bary_coord_p20[0] + t1.v() * bary_coord_p20[1] +
							  t2.v() * bary_coord_p20[2];
					t20.n() = t0.n();

					Face neg_face_1 = { { v0, v1, v12 }, { t0, t1, t12 } };
					negative.push_back(neg_face_1);

					Face neg_face_2 = { { v0, v12, v20 }, { t0, t12, t20 } };
					negative.push_back(neg_face_2);

					Face pos_face_1 = { { v2, v20, v12 }, { t2, t20, t12 } };
					positive.push_back(pos_face_1);
				}

				void SplitTrianglePMZ(FaceArray& negative,
									  FaceArray& positive,
									  int v0,
									  int v1,
									  int v2,
									  vcg::TexCoord2<float>& t0,
									  vcg::TexCoord2<float>& t1,
									  vcg::TexCoord2<float>& t2)
				{
					CHECK(t0.n() == t1.n() && t1.n() == t2.n());

					int v01 = edges_map[EdgeKey(v0, v1)].second;

					const auto& p0 = clip_vertices[v0];
					const auto& p1 = clip_vertices[v1];
					const auto& p2 = clip_vertices[v2];
					const auto& p01 = clip_vertices[v01];

					vcg::Point3<Real> bary_coord_p01;
					barycentric_coordinates(p01, p0, p1, p2, bary_coord_p01);

					vcg::TexCoord2<float> t01;
					t01.u() = t0.u() * bary_coord_p01[0] + t1.u() * bary_coord_p01[1] +
							  t2.u() * bary_coord_p01[2];
					t01.v() = t0.v() * bary_coord_p01[0] + t1.v() * bary_coord_p01[1] +
							  t2.v() * bary_coord_p01[2];
					t01.n() = t0.n();

					Face pos_face_1 = { { v2, v0, v01 }, { t2, t0, t01 } };
					positive.push_back(pos_face_1);

					Face neg_face_1 = { { v2, v01, v1 }, { t2, t01, t1 } };
					negative.push_back(neg_face_1);
				}

				void SplitTriangleMPZ(FaceArray& negative,
									  FaceArray& positive,
									  int v0,
									  int v1,
									  int v2,
									  vcg::TexCoord2<float>& t0,
									  vcg::TexCoord2<float>& t1,
									  vcg::TexCoord2<float>& t2)
				{
					CHECK(t0.n() == t1.n() && t1.n() == t2.n());

					int v01 = edges_map[EdgeKey(v0, v1)].second;

					const auto& p0 = clip_vertices[v0];
					const auto& p1 = clip_vertices[v1];
					const auto& p2 = clip_vertices[v2];
					const auto& p01 = clip_vertices[v01];

					vcg::Point3<Real> bary_coord_p01;
					barycentric_coordinates(p01, p0, p1, p2, bary_coord_p01);

					vcg::TexCoord2<float> t01;
					t01.u() = t0.u() * bary_coord_p01[0] + t1.u() * bary_coord_p01[1] +
							  t2.u() * bary_coord_p01[2];
					t01.v() = t0.v() * bary_coord_p01[0] + t1.v() * bary_coord_p01[1] +
							  t2.v() * bary_coord_p01[2];
					t01.n() = t0.n();

					Face neg_face_1 = { { v2, v0, v01 }, { t2, t0, t01 } };
					negative.push_back(neg_face_1);

					Face pos_face_1 = { { v2, v01, v1 }, { t2, t01, t1 } };
					positive.push_back(pos_face_1);
				}


				void CreateMesh(const FaceArray& faces, CMeshO& mesh)
				{
					// 创建网格
					std::map<Point3m, size_t> mapVertices;

					for (size_t i = 0; i < faces.size(); ++i)
					{
						const Face& face = faces[i];

						// 添加顶点
						size_t vertexId[3];

						for (int v = 0; v < 3; ++v)
						{
							const auto& p = clip_vertices[face.v[v]];

							if (mapVertices.find(p) == mapVertices.end())
							{
								mapVertices[p] = mesh.vert.size();
								vcg::tri::Allocator<CMeshO>::AddVertex(mesh, p);
							}

							vertexId[v] = mapVertices[p];
						}

						auto newfi = vcg::tri::Allocator<CMeshO>::AddFace(mesh,
																		  vertexId[0],
																		  vertexId[1],
																		  vertexId[2]);

						if (newfi->IsWedgeTexCoordEnabled())
						{
							for (int v = 0; v < 3; ++v)
								newfi->WT(v) = face.tex[v];
						}

						if (newfi->IsColorEnabled())
							newfi->C() = vcg::Color4b(vcg::Color4b::White);
					}
				}
			};
		}

		//////////////////////////////////////////////////////////////////////////
		CMeshO::CMeshO() : vcgTriMesh()
		{
			currentDataMask = MM_NONE;
			currentDataMask |= MM_VERTCOORD | MM_VERTNORMAL | MM_VERTFLAG;
			currentDataMask |= MM_FACEVERT | MM_FACENORMAL | MM_FACEFLAG;
		}

		CMeshO::CMeshO(CMeshO&& oth) : vcgTriMesh(), currentDataMask(oth.currentDataMask)
		{
			enableOCFComponentsFromOtherMesh(oth);
			vcg::tri::Append<vcgTriMesh, vcgTriMesh>::Mesh(*this, oth);
		}

		CMeshO::CMeshO(CMeshO& oth)
		{
			enableOCFComponentsFromOtherMesh(oth);
			tri::Append<CMeshO, CMeshO>::Mesh(*this, oth);
			bbox.Import(oth.bbox);
		}

		/*CMeshO& CMeshO::operator=(CMeshO&& oth)
		{
			Clear();
			enableOCFComponentsFromOtherMesh(oth);
			vcg::tri::Append<vcgTriMesh, vcgTriMesh>::MeshCopy(*this, oth);
			currentDataMask = oth.currentDataMask;
			return *this;
		}*/

		CMeshO& CMeshO::operator=(CMeshO& oth)
		{
			Clear();
			enableOCFComponentsFromOtherMesh(oth);
			vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(*this, oth);
			currentDataMask = oth.currentDataMask;
			return *this;
		}

		void CMeshO::UpdateBoxAndNormals()
		{
			// 更新边界、法线
			vcg::tri::UpdateBounding<CMeshO>::Box(*this);

			if (this->fn > 0)
			{
				vcg::tri::UpdateNormal<CMeshO>::PerFaceNormalized(*this);
				vcg::tri::UpdateNormal<CMeshO>::PerVertexAngleWeighted(*this);
			}
		}

		bool CMeshO::ClipWithPlanar(const Plane3m& clipingPlane)
		{
			SplitMeshByPlane splitter;
			splitter(*this, clipingPlane);

			*this = splitter.positive_mesh;
			return true;
		}

		bool CMeshO::ClipExtents(const Point3m& min, const Point3m& max)
		{
			// 提取范围内的网格，缩小网格数据规模
			double radius = (max - min).Norm() * 0.5;
			Point3m center = (max + min) * 0.5;


			bool clip_success = true;

			std::vector<Plane3m> planes;
			CreateClipPlanes(min, max, planes);

			for (auto& plane : planes)
			{
				// CMeshO under, over;
				if (!ClipWithPlanar(plane))
				{
					clip_success = false;
					break;
				}
			}

			return clip_success;
		}

		void CMeshO::enableOCFComponentsFromOtherMesh(const CMeshO& oth)
		{
			// vertex
			if (oth.vert.IsVFAdjacencyEnabled())
				this->vert.EnableVFAdjacency();
			if (oth.vert.IsMarkEnabled())
				this->vert.EnableMark();
			if (oth.vert.IsTexCoordEnabled())
				this->vert.EnableTexCoord();
			if (oth.vert.IsCurvatureEnabled())
				this->vert.EnableCurvature();
			if (oth.vert.IsCurvatureDirEnabled())
				this->vert.EnableCurvatureDir();
			if (oth.vert.IsRadiusEnabled())
				this->vert.EnableRadius();

			// face
			if (oth.face.IsQualityEnabled())
				this->face.EnableQuality();
			if (oth.face.IsMarkEnabled())
				this->face.EnableMark();
			if (oth.face.IsColorEnabled())
				this->face.EnableColor();
			if (oth.face.IsFFAdjacencyEnabled())
				this->face.EnableFFAdjacency();
			if (oth.face.IsVFAdjacencyEnabled())
				this->face.EnableVFAdjacency();
			if (oth.face.IsCurvatureDirEnabled())
				this->face.EnableCurvatureDir();
			if (oth.face.IsWedgeTexCoordEnabled())
				this->face.EnableWedgeTexCoord();
		}

		//////////////////////////////////////////////////////////////////////////
		void CreateClipPlanes(const Point3m& min, const Point3m& max, std::vector<Plane3m>& planes)
		{
			planes.clear();

#if 0
			// plane 1
			Point3m center_1 = min;
			Point3m normal_1(0, 1, 0);

			Plane3m plane_1;
			plane_1.Init(center_1, normal_1);
			planes.push_back(plane_1);

			// plane 2
			Point3m center_2 = min;
			Point3m normal_2(1, 0, 0);

			Plane3m plane_2;
			plane_2.Init(center_2, normal_2);
			planes.push_back(plane_2);

			// plane 3
			Point3m center_3 = max;
			Point3m normal_3(0, -1, 0);

			Plane3m plane_3;
			plane_3.Init(center_3, normal_3);
			planes.push_back(plane_3);

			// plane 4
			Point3m center_4 = max;
			Point3m normal_4(-1, 0, 0);

			Plane3m plane_4;
			plane_4.Init(center_4, normal_4);
			planes.push_back(plane_4);
#else
			// plane 1
			Point3m center_1 = min;
			Point3m normal_1(0, 0, 1);

			Plane3m plane_1;
			plane_1.Init(center_1, normal_1);
			planes.push_back(plane_1);

			// plane 2
			Point3m center_2 = max;
			Point3m normal_2(0, 0, -1);

			Plane3m plane_2;
			plane_2.Init(center_2, normal_2);
			planes.push_back(plane_2);

#endif
		}
	}
}