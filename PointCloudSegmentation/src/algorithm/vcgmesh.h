/////////////////////////////////////////////////////////////////////
// 文件名称：vcgmesh.h
// 功能描述：VCG网格模型包装类
// 创建标识：吕伟	2022/12/26
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#ifndef VCGMESH_H_
#define VCGMESH_H_

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/topology.h>

#include <wrap/io_trimesh/io_mask.h>
#include <wrap/io_trimesh/export_stl.h>

#define PCS_SCALAR double

#ifndef PCS_SCALAR
#error "Fatal compilation error: PCS_SCALAR must be defined"
#endif

// 重用数据类型语义定义
typedef PCS_SCALAR Scalarm;

typedef vcg::Point2<PCS_SCALAR> Point2m;
typedef vcg::Point3<PCS_SCALAR> Point3m;
typedef vcg::Point4<PCS_SCALAR> Point4m;
typedef vcg::Plane3<PCS_SCALAR> Plane3m;
typedef vcg::Segment2<PCS_SCALAR> Segment2m;
typedef vcg::Segment3<PCS_SCALAR> Segment3m;
typedef vcg::Box3<PCS_SCALAR> Box3m;
typedef vcg::Matrix44<PCS_SCALAR> Matrix44m;
typedef vcg::Matrix33<PCS_SCALAR> Matrix33m;
typedef vcg::Shot<PCS_SCALAR> Shotm;
typedef vcg::Similarity<PCS_SCALAR> Similaritym;

namespace vcg {
	namespace vertex {
		template <class T>
		class Coord3m : public Coord<vcg::Point3<Scalarm>, T>
		{
		public:
			static void Name(std::vector<std::string>& name)
			{
				name.push_back(std::string("Coord3m"));
				T::Name(name);
			}
		};

		template <class T>
		class Normal3m : public Normal<vcg::Point3<Scalarm>, T>
		{
		public:
			static void Name(std::vector<std::string>& name)
			{
				name.push_back(std::string("Normal3m"));
				T::Name(name);
			}
		};

		template <class T>
		class CurvatureDirmOcf : public CurvatureDirOcf<CurvatureDirTypeOcf<Scalarm>, T>
		{
		public:
			static void Name(std::vector<std::string>& name)
			{
				name.push_back(std::string("CurvatureDirmOcf"));
				T::Name(name);
			}
		};

		template <class T>
		class RadiusmOcf : public RadiusOcf<Scalarm, T>
		{
		public:
			static void Name(std::vector<std::string>& name)
			{
				name.push_back(std::string("RadiusmOcf"));
				T::Name(name);
			}
		};
	} // end namespace vertex

	namespace face {
		template <class T>
		class Normal3m : public NormalAbs<vcg::Point3<Scalarm>, T>
		{
		public:
			static void Name(std::vector<std::string>& name)
			{
				name.push_back(std::string("Normal3m"));
				T::Name(name);
			}
		};

		template <class T>
		class CurvatureDirmOcf : public CurvatureDirOcf<CurvatureDirOcfBaseType<Scalarm>, T>
		{
		public:
			static void Name(std::vector<std::string>& name)
			{
				name.push_back(std::string("CurvatureDirdOcf"));
				T::Name(name);
			}
		};
	}
}

namespace d3s {
	namespace pcs {


		// Texture Patch Component
		struct TextureComponent
		{
			int textureIdx;
			int patchIdx;
			double minx, miny;
			double maxx, maxy;
		};

		typedef std::vector<TextureComponent> TextureComponents;

		// VCG Mesh
		//////////////////////////////////////////////////////////////////////////
		enum MeshElement
		{
			MM_NONE = 0x00000000,
			MM_VERTCOORD = 0x00000001,
			MM_VERTNORMAL = 0x00000002,
			MM_VERTFLAG = 0x00000004,
			MM_VERTCOLOR = 0x00000008,
			MM_VERTQUALITY = 0x00000010,
			MM_VERTMARK = 0x00000020,
			MM_VERTFACETOPO = 0x00000040,
			MM_VERTCURV = 0x00000080,
			MM_VERTCURVDIR = 0x00000100,
			MM_VERTRADIUS = 0x00000200,
			MM_VERTTEXCOORD = 0x00000400,
			MM_VERTNUMBER = 0x00000800,

			MM_FACEVERT = 0x00001000,
			MM_FACENORMAL = 0x00002000,
			MM_FACEFLAG = 0x00004000,
			MM_FACECOLOR = 0x00008000,
			MM_FACEQUALITY = 0x00010000,
			MM_FACEMARK = 0x00020000,
			MM_FACEFACETOPO = 0x00040000,
			MM_FACENUMBER = 0x00080000,
			MM_FACECURVDIR = 0x00100000,

			MM_WEDGTEXCOORD = 0x00200000,
			MM_WEDGNORMAL = 0x00400000,
			MM_WEDGCOLOR = 0x00800000,

			// 	Selection
			MM_VERTFLAGSELECT = 0x01000000,
			MM_FACEFLAGSELECT = 0x02000000,

			// Per Mesh Stuff....
			MM_CAMERA = 0x08000000,
			MM_TRANSFMATRIX = 0x10000000,
			MM_COLOR = 0x20000000,
			MM_POLYGONAL = 0x40000000,
			MM_UNKNOWN = 0x80000000,

			MM_ALL = 0xffffffff
		};

		template <typename MeshType>
		void UpdateDataMask(MeshType& cm, int neededDataMask)
		{
			if ((neededDataMask & MM_FACEFACETOPO) != 0)
			{
				cm.face.EnableFFAdjacency();
				vcg::tri::UpdateTopology<MeshType>::FaceFace(cm);
			}

			if ((neededDataMask & MM_VERTFACETOPO) != 0)
			{
				cm.vert.EnableVFAdjacency();
				cm.face.EnableVFAdjacency();
				vcg::tri::UpdateTopology<MeshType>::VertexFace(cm);
			}

			if ((neededDataMask & MM_WEDGTEXCOORD) != 0)
				cm.face.EnableWedgeTexCoord();

			if ((neededDataMask & MM_FACECOLOR) != 0)
				cm.face.EnableColor();

			if ((neededDataMask & MM_FACEQUALITY) != 0)
				cm.face.EnableQuality();

			if ((neededDataMask & MM_FACECURVDIR) != 0)
				cm.face.EnableCurvatureDir();

			if ((neededDataMask & MM_FACEMARK) != 0)
				cm.face.EnableMark();

			if ((neededDataMask & MM_VERTMARK) != 0)
				cm.vert.EnableMark();

			if ((neededDataMask & MM_VERTCURV) != 0)
				cm.vert.EnableCurvature();

			if ((neededDataMask & MM_VERTCURVDIR) != 0)
				cm.vert.EnableCurvatureDir();

			if ((neededDataMask & MM_VERTRADIUS) != 0)
				cm.vert.EnableRadius();

			if ((neededDataMask & MM_VERTTEXCOORD) != 0)
				cm.vert.EnableTexCoord();

			cm.currentDataMask |= neededDataMask;
		}

		template <typename MeshType>
		static bool HasDataMask(MeshType& cm, const int maskToBeTested)
		{
			return ((cm.currentDataMask & maskToBeTested) != 0);
		}

		template <typename MeshType>
		static void ClearDataMask(MeshType& cm, int unneededDataMask)
		{
			if (((unneededDataMask & MM_VERTFACETOPO) != 0) && HasDataMask(cm, MM_VERTFACETOPO))
			{
				cm.face.DisableVFAdjacency();
				cm.vert.DisableVFAdjacency();
			}
			if (((unneededDataMask & MM_FACEFACETOPO) != 0) && HasDataMask(cm, MM_FACEFACETOPO))
				cm.face.DisableFFAdjacency();

			if (((unneededDataMask & MM_WEDGTEXCOORD) != 0) && HasDataMask(cm, MM_WEDGTEXCOORD))
				cm.face.DisableWedgeTexCoord();

			if (((unneededDataMask & MM_FACECOLOR) != 0) && HasDataMask(cm, MM_FACECOLOR))
				cm.face.DisableColor();

			if (((unneededDataMask & MM_FACEQUALITY) != 0) && HasDataMask(cm, MM_FACEQUALITY))
				cm.face.DisableQuality();

			if (((unneededDataMask & MM_FACEMARK) != 0) && HasDataMask(cm, MM_FACEMARK))
				cm.face.DisableMark();

			if (((unneededDataMask & MM_VERTMARK) != 0) && HasDataMask(cm, MM_VERTMARK))
				cm.vert.DisableMark();

			if (((unneededDataMask & MM_VERTCURV) != 0) && HasDataMask(cm, MM_VERTCURV))
				cm.vert.DisableCurvature();

			if (((unneededDataMask & MM_VERTCURVDIR) != 0) && HasDataMask(cm, MM_VERTCURVDIR))
				cm.vert.DisableCurvatureDir();

			if (((unneededDataMask & MM_VERTRADIUS) != 0) && HasDataMask(cm, MM_VERTRADIUS))
				cm.vert.DisableRadius();

			if (((unneededDataMask & MM_VERTTEXCOORD) != 0) && HasDataMask(cm, MM_VERTTEXCOORD))
				cm.vert.DisableTexCoord();

			cm.currentDataMask = cm.currentDataMask & (~unneededDataMask);
		}


		// 顶点、边、面 类型的前向声明
		class CVertexO;
		class CEdgeO;
		class CFaceO;

		// 使用数据类型的语义
		class CUsedTypesO : public vcg::UsedTypes<vcg::Use<CVertexO>::AsVertexType,
												  vcg::Use<CEdgeO>::AsEdgeType,
												  vcg::Use<CFaceO>::AsFaceType>
		{
		public:
			int ref_index;
		};

		// 顶点类型适配
		class CVertexO : public vcg::Vertex<CUsedTypesO,
											vcg::vertex::InfoOcf,		   /*  4b */
											vcg::vertex::Coord3m,		   /* 12b */
											vcg::vertex::BitFlags,		   /*  4b */
											vcg::vertex::Normal3m,		   /* 12b */
											vcg::vertex::Qualityf,		   /*  4b */
											vcg::vertex::Color4b,		   /*  4b */
											vcg::vertex::VFAdjOcf,		   /*  0b */
											vcg::vertex::MarkOcf,		   /*  0b */
											vcg::vertex::TexCoordfOcf,	   /*  0b */
											vcg::vertex::CurvaturefOcf,	   /*  0b */
											vcg::vertex::CurvatureDirmOcf, /*  0b */
											vcg::vertex::RadiusmOcf		   /*  0b */
											>
		{
		};


		// 边类型适配
		class CEdgeO : public vcg::Edge<CUsedTypesO,
										vcg::edge::BitFlags, /*  4b */
										vcg::edge::EVAdj,
										vcg::edge::EEAdj>
		{
		};

		// 面类型适配。对于每个面，32位系统需要32个字节，64位系统需要48个字节
		class CFaceO : public vcg::Face<CUsedTypesO,
										vcg::face::InfoOcf,			 /* 4b */
										vcg::face::VertexRef,		 /*12b */
										vcg::face::BitFlags,		 /* 4b */
										vcg::face::Normal3m,		 /*12b */
										vcg::face::QualityfOcf,		 /* 0b */
										vcg::face::MarkOcf,			 /* 0b */
										vcg::face::Color4bOcf,		 /* 0b */
										vcg::face::FFAdjOcf,		 /* 0b */
										vcg::face::VFAdjOcf,		 /* 0b */
										vcg::face::CurvatureDirmOcf, /* 0b */
										vcg::face::WedgeTexCoordfOcf /* 0b */
										>
		{
		};

		typedef vcg::tri::TriMesh<vcg::vertex::vector_ocf<CVertexO>, vcg::face::vector_ocf<CFaceO>>
			vcgTriMesh;

		class CMeshO : public vcgTriMesh
		{
		public:
			int currentDataMask;

			enum SeamAwareDegree
			{
				NoUVShapePreserving,
				UVShapePreserving,
				Seamless
			};

			CMeshO();

			CMeshO(CMeshO& oth);

			CMeshO(CMeshO&& oth);

			CMeshO& operator=(CMeshO& oth);

			// CMeshO& operator=(CMeshO&& oth);

			/**
			 *  @brief    更新边和法线
			 *
			 *  @return   void
			 */
			void UpdateBoxAndNormals();

			/**
			 *  @brief    通过平面裁剪，保留裁剪面内侧(沿平面法线方向)的网格。
			 *
			 *  @param    const Plane3m & clipingPlane	裁剪平面
			 *
			 *  @return   CMeshO&				裁剪后的网格
			 */
			bool ClipWithPlanar(const Plane3m& clipingPlane);


			/**
			 *  @brief    通过边界框裁剪网格
			 *
			 *  @prarm	 const Point3m & min			边界框左下角点
			 *  @prarm	 const Point3m & max			边界框左下角点
			 *
			 *  @return   bool
			 */
			bool ClipExtents(const Point3m& min, const Point3m& max);


		private:
			void enableOCFComponentsFromOtherMesh(const CMeshO& oth);
		};

		/**
		 *  @brief    通过边界创建裁剪面
		 *
		 *  @prarm	 const Point3m & min				边界框左下角点
		 *  @prarm	 const Point3m & max				边界框右上角点
		 *  @prarm	 std::vector<Plane3m> & planes		裁剪平面
		 *
		 *  @return   void
		 */
		void CreateClipPlanes(const Point3m& min, const Point3m& max, std::vector<Plane3m>& planes);
	}
}

#endif // VCGMESH_H_