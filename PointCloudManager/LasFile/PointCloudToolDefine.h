//*****************************************************
//
//    @copyright      	三维技术部
//    @version        	v1.0
//    @file           	PointCloudToolDefine.H
//    @author         	LC
//    @data           	2022/6/28 20:15
//    @brief          	PointCloudTools定义文件
//*****************************************************
#ifndef POINTCLOUDTOOLDEFINE_H_
#define POINTCLOUDTOOLDEFINE_H_

#include <include/PointCloudManagerExport.h>
#include <include/ReferenceCountObj.h>
#include <include/StringToolkit.h>
#include <include/share_ptr.h>

#include <osgDB/Options>

namespace pc {
	namespace data {
		// 文件读写option
		static const osgDB::Options S_IVE_OPTION = osgDB::Options(); // 方便后续加密替换
		const std::string TexPostStr = "_tex";						 // 纹理文件的后缀
		const std::string ColorPostStr = "_color";					 // 颜色文件的后缀
		const std::string strFilExt = ".ive";						 // ive文件后缀

		///////////////////////////////////////////////////////////////////////点云抽稀层级///////////////////////////////////////////////////////////////////////
		enum EPointCloudSimplifyLevel
		{
			eOneLecel = 0,	 // 抽稀第一层级
			eTwoLevel = 1,	 // 抽稀第二层级
			eThreeLevel = 2, // 抽稀第三层级
		};

		/////////////////////////////////////////////////////////////////////////点云信息/////////////////////////////////////////////////////////////////////////
		// 类型纹理数组索引
		static const int S_INT_TYPE_TEXCOORD_INDEX = 0;

		static const char* S_STRING_MODEL_MATRIX_UNIFORM = "matModelMatrix";

		struct POINTCLOUDMANAGER_EXPORT PointCloudInfo
		{
			PointCloudInfo(void) {}
			PointCloudInfo(const std::vector<osg::Vec3>& vVertexs,
						   const std::vector<osg::Vec4ub>& vColors,
						   const std::vector<osg::Vec2>& vTypeTexCoords)
				: vVertexs(vVertexs), vColors(vColors), vTypeTexCoords(vTypeTexCoords)
			{
			}

			std::vector<osg::Vec3> vVertexs;	   // 顶点列表
			std::vector<osg::Vec4ub> vColors;	   // 颜色列表
			std::vector<osg::Vec2> vTypeTexCoords; // 类型纹理数组列表（第0个纹理数组）
		};

		// 属性操作遍历器参数
		struct SVisitorCallbackParam
		{
			SVisitorCallbackParam()
				: _nIndex(-1), _bExitTraversal(false), _pAdditionalParam(nullptr)
			{
			}
			osg::NodePath _nodePath;
			osg::ref_ptr<osg::Vec2Array> _pTexCoordArray;
			size_t _nIndex;
			bool _bExitTraversal;
			osg::Vec3d _point;
			d3s::share_ptr<d3s::ReferenceCountObj> _pAdditionalParam;
			CString _strPageNodeId; // pagenode id
		};
		// 属性操作遍历器回调
		typedef bool (*PropVisitorCallback)(SVisitorCallbackParam& callbackParam);

		// 圆
		struct SCircularParam
		{
			SCircularParam() : _dR(0.0) {}

			osg::Vec3d _center; // 圆心
			double _dR;			// 半径
		};

		// 遍历信息
		struct POINTCLOUDMANAGER_EXPORT SLimitValue
		{
			SLimitValue(const int& nMaxX = 0,
						const int& nMinX = 0,
						const int& nMaxY = 0,
						const int& nMinY = 0)
				: _nMaxX(nMaxX), _nMinX(nMinX), _nMaxY(nMaxY), _nMinY(nMinY)
			{
			}
			int _nMaxX;
			int _nMinX;
			int _nMaxY;
			int _nMinY;
		};
		typedef std::map<int, std::pair<SLimitValue, std::vector<bool>>> FinePolygonMap;
		struct POINTCLOUDMANAGER_EXPORT SVisitorInfo
		{
			// 0(多边形外)，1(多边形内)，2(边)
			static const char g_cOutPolygon = 0;
			static const char g_cInPolygon = 1;
			static const char g_cPolygonBoundary = 2;

			std::vector<char> _coarsePolygon;			  // 粗糙多边拟合值：
			osg::Matrix _coarseVPW;						  // 粗糙视口vpw矩阵
			std::vector<osg::Vec3d> _coarsePolygonPoints; // 粗糙多边形点集
			SLimitValue _coarsePolygonLimit;			  // 粗糙多边形极值
			osg::Matrix _fineVPW;						  // 精细视口vpw矩阵
			FinePolygonMap _finePolygonMap; // 精细多边形拟合值：true(多边形外)，false(多边形内)

			SCircularParam _circularParam; // 圆
			osg::BoundingBox _boundingBox; // 包围盒
		};
		struct POINTCLOUDMANAGER_EXPORT SVisitorInfos
		{
			enum EVisitorType
			{
				ePolygon,		// 多边形遍历查找
				eBoundingBoxXY, // XY平面二维包围盒遍历查找
				eBoundingBox,	// 包围盒遍历查找
				ePolygonXY,		// XY平面二维多边形遍历查找
				eCircular		// XY平面二维圆形遍历查找
			};

			SVisitorInfos() : _pBoundingBox(new osg::BoundingBox()) { Init(); }
			SVisitorInfos(SVisitorInfo visitorInfo,
						  SVisitorInfo sliceVisitorInfo = SVisitorInfo{},
						  const std::vector<unsigned>& hideSegmentList = std::vector<unsigned>{})
				: _visitorInfo(visitorInfo),
				  _hideSegmentList(hideSegmentList),
				  _sliceVisitorInfo(sliceVisitorInfo),
				  _pBoundingBox(new osg::BoundingBox())
			{
				Init();
			}
			void Init()
			{
				_visitorType = ePolygon;
				_pPropVisitorCallback = nullptr;
				_pAdditionalParam = nullptr;
				_bAllTraversal = false;
			}
			void ExpandBoundingBox(const osg::Vec3d& point)
			{
				if (nullptr == _pBoundingBox)
					return;
				_pBoundingBox->expandBy(point);
			}
			osg::BoundingBox GetBoundingBox(void)
			{
				if (nullptr == _pBoundingBox)
					return osg::BoundingBox();
				return *_pBoundingBox;
			}

			EVisitorType _visitorType;								  // 遍历类型
			SVisitorInfo _visitorInfo;								  // 遍历参数
			SVisitorInfo _sliceVisitorInfo;							  // 剖切视口遍历信息
			std::vector<unsigned> _hideSegmentList;					  // 隐藏分类列表
			PropVisitorCallback _pPropVisitorCallback;				  // 回调函数指针
			d3s::share_ptr<d3s::ReferenceCountObj> _pAdditionalParam; // 回调附加参数
			bool _bAllTraversal;									  // 是否全遍历
		private:
			osg::BoundingBox* _pBoundingBox; // 遍历操作点的包围盒
		};

		// 多边形点填充参数
		struct SPolygonFillParam
		{
			double _dScanLineY;									// 扫描线y坐标
			double _dStep;										// 填充间距
			std::vector<std::pair<double, double>> _scanResult; // 扫描结果区间
			double _dZ;											// 填充高程
		};
		typedef std::vector<SPolygonFillParam> PolygonFillParamVec;

		// 点云切割点
		struct SSplitPos : public d3s::ReferenceCountObj
		{
			SSplitPos()
			{
				strTargetLasFile = L"";
				strTarget3DTilesPath = L"";
				bVaild = false;
				dArea = 0;
				dPcCount = 0;
			};
			virtual ~SSplitPos(void) {};

			osg::Vec3d towerPos;		  // 切割点（塔位置）
			CString strTargetLasFile;	  // 生成目标las文件，全路径
			CString strTarget3DTilesPath; // 生成3DTiles目录
			CString strTowerId;			  // 杆塔Id
			bool bVaild;				  // 是否在点云中
			bool bLasSuccess;			  // las是否成功
			double dArea;				  // 点云密度
			double dPcCount;			  // 点云密度
		};

		typedef std::vector<d3s::share_ptr<SSplitPos>> SplitPosVec;

	}
}

#endif // POINTCLOUDTOOLDEFINE_H_