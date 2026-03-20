#ifndef OSGDEFINES_H_
#define OSGDEFINES_H_

#include <LasFile/PointCloudToolDefine.h>
#include <LasFile/PointCloudPageLod.h>

#include <include/CommonToolsDef.h>
#include <include/ModelNode.h>

#include <pcl/common/common.h>

#include <tbb/mutex.h>


#define EPSILON 10e-6
#define BIG_EPSILON 10e-5
#define GEOM_INFINITE (2.0e+100)
#define CP_ACP 0

enum EPageLodLevel
{
	eZeroLevelPageLod = 0, // 精细层级
	eOneLevelPageLod = 1,  // 抽稀1层级
	eTwoLevelPageLod = 2,  // 抽稀2层级
	eWideLevelPageLod = 3, // 粗糙层级
	eTitlePageLod = 4,	   // 标题层级（用于包含以下层级的根节点）
};

BEGIN_ENUM_CONVERSION(EPageLodLevel)
{
	{ eZeroLevelPageLod, _T("Zero") }, { eOneLevelPageLod, _T("One") },
		{ eTwoLevelPageLod, _T("Two") }, { eWideLevelPageLod, _T("Three") },
	{
		NULL, NULL
	}
}
END_ENUM_CONVERSION(EPageLodLevel);


#define MAX_MEMORY_POINT_NUM 40000000
#define PIECE_SAVE_NUM 500000
#define OctteeSize 10
#define PieceMinSize 125.0
#define PieceOneSize 250.00
#define PieceTwoSize 1000.00
#define PieceLasSize 1000.0
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudPtr;
typedef std::pair<double, double> minmax_t;
typedef pcl::PointXYZRGBA PointT;
struct MoudelInfo
{
	MoudelInfo() : nPointCnt(0)
	{
		strModelLevel = L"";
		strFileName = L"";
		strPageLodID = L"";
		dMinx = 0.0;
		dMinY = 0.0;
		dMinZ = 0.0;
		dMaxX = 0.0;
		dMaxY = 0.0;
		dMaxZ = 0.0;
	}

	CString strModelLevel; // 模型层级
	CString strFileName;   // 文件名称
	CString strPageLodID;  // 对应PaGeLodID
	// 模型包络框的最大最小坐标
	double dMinx;
	double dMinY;
	double dMinZ;
	double dMaxX;
	double dMaxY;
	double dMaxZ;
	size_t nPointCnt; // 点数量
};

// 切片后的一个块数据（正方形）
namespace LasOsg {
	struct PieceInfo : public d3s::ReferenceCountObj
	{
		PieceInfo()
		{
			pFatherPiece = nullptr;
			nIdex = 0;
			nX = 0;
			nY = 0;
			nZ = 0;
			nWidth = 0;
			nHeight = 0;
			nScale = 0.0;
			nLevel = 0;
		}
		~PieceInfo() {}
		CString strOsgFileName;	 // 对应的OSG文件名称
		CString strLasName;		 // las文件名称
		CString strName;		 // 分块名称“000-000”
		CString strRelativePath; // 相对输出目录的相对路径
		CString strOsgRePath;
		CloudPtr cloudPt;									  // 当期块的点云
		std::vector<CString> childOsgFileNameVec;			  // 子块osg文件名称
		CString strOgbPath;									  // osg文件名称
		std::vector<d3s::share_ptr<PieceInfo>> ChildPieceVec; // 子级块的map，key为子级块name
		PieceInfo* pFatherPiece;							  // 父级块
		std::vector<std::string> allPCDFilePath;			  // 每个块中50万个点写一个文件集合
		std::string childName; // 直接子节点名称，50万个点抽稀后的上一级节点对应名称
		std::vector<pc::data::CModelNodePtr> childpagedLodPrtvec; // 子节点pageLod模型指针
		// osg::ref_ptr<CPointCloudpagedLod>	pPieceInfoPagedLod;	//当前切片对应的pagedlod
		pc::data::CModelNodePtr pPieceInfoPagedLod;
		osg::Vec3d minPoint; // 块的最小点（已做相对原点的偏移）
		osg::Vec3d maxPoint; // 块的最大点（已做相对原点的偏移）
		int nIdex;			 // 每个块中50万个点的索引值
		int nX;				 // 当前块的最小X值
		int nY;				 // 当前块的最小Y值
		int nZ;
		int nWidth; // 当前块宽度
		int nHeight;
		double nScale; // 当前块采样比例
		int nLevel;	   // 当前块所属层级

		tbb::mutex mtx;
	};
}

// 切片后的一个块数据（正方形）
namespace LasSplit {
	struct SplitInfo : public d3s::ReferenceCountObj
	{
		SplitInfo(d3s::share_ptr<pc::data::SSplitPos> pSplitPos)
		{
			_pSplitPos = pSplitPos;
			nIdex = 0;
			_strPcdPath = L"";
			bTop = false;
			bBottom = false;
			bLeft = false;
			bRight = false;
		}
		~SplitInfo() {}
		osg::Vec2d _center;								// 杆塔中心点，相对点云中心点
		osg::Vec3d _offsetXyz;							// 点云中心点
		CloudPtr cloudPt;								// 当期块的点云
		CString _strPcdPath;							// PCD文件存储目录
		CString _str3DTilesPath;						// 3DTiles存储目录
		std::vector<std::string> allPCDFilePath;		// 每个块中50万个点写一个文件集合
		int nIdex;										// 每个块中50万个点的索引值
		d3s::share_ptr<pc::data::SSplitPos> _pSplitPos; // 对应的切割点

		// 上下左右全为true，则有效
		bool bTop;					   // 上
		bool bBottom;				   // 下
		bool bLeft;					   // 左
		bool bRight;				   // 右
		osg::BoundingBox _boundingBox; // 包围盒
		int _nEpsg;					   // 度带
		CString _strScanTime;		   // 点云扫描时间
	};
}

#endif // OSGDEFINES_H_