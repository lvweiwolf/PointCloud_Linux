#ifndef POINTCLOUDMANAGERDEF_H_
#define POINTCLOUDMANAGERDEF_H_

#include <include/Log.h>
#include <LasFile/OsgDefines.h>

#define MASK(x) (1 << (x))

namespace pc {
	template <typename T>
	using share_ptr = typename d3s::share_ptr<T>;

	namespace data {
		// PagedLod关联文件
		struct tagPagedLodFile
		{
			CString strPointInfoFile; // 点信息文件
			CString strPointTexFile;  // 点纹理文件

			friend bool operator<(const tagPagedLodFile& object1, const tagPagedLodFile& object2)
			{
				return object1.strPointInfoFile.Compare(object2.strPointInfoFile) +
						   object1.strPointTexFile.Compare(object2.strPointTexFile) <
					   0;
			}
		};

		// 点云元素唯一id
		static const TCHAR* POINTCLOUD_UNIQUEID = L"POINTCLOUD_UNIQUEID";
		// 水系元素唯一id
		static const TCHAR* WATERSYS_UNIQUEID = L"WATERSYS_UNIQUEID";
		// 导线修复元素唯一id
		static const TCHAR* WIREREPAIR_UNIQUEID = L"WATERSYS_UNIQUEID";
		// 模型类型
		static const std::string STRING_MODEL_TYPE_NAME = "MODEL_TYPE";
		// 点云模型
		static const std::string STRING_LAS_MODEL = "LAS_MODEL";
		// 点云分类类型支持数量
		static const int N_POINT_CLOUD_TYPE_SUPPORT_SIZE = 255;

		// 二维区域包围盒
		struct PointCloudBoundBox2D
		{
			PointCloudBoundBox2D() : _dXmin(0.0), _dXmax(0.0), _dYmin(0.0), _dYmax(0.0) {}
			PointCloudBoundBox2D(double dXmin, double dXmax, double dYmin, double dYmax)
				: _dXmin(dXmin), _dXmax(dXmax), _dYmin(dYmin), _dYmax(dYmax)
			{
			}
			PointCloudBoundBox2D(const PointCloudBoundBox2D& rhs)
				: _dXmin(rhs._dXmin), _dXmax(rhs._dXmax), _dYmin(rhs._dYmin), _dYmax(rhs._dYmax)
			{
			}

			// 判断包围盒是否有相交区域
			bool IsIntersect(const PointCloudBoundBox2D& rhs) const
			{
				if (_dXmax <= rhs._dXmin || _dXmin >= rhs._dXmax || _dYmax <= rhs._dYmin ||
					_dYmin >= rhs._dYmax)
					return false;

				return true;
			}

			/**
			 *  函数介绍    	判断点是否在包围盒内
			 *
			 *  输入参数    	double dX			点的x坐标
			 *  输入参数    	double dY			点的y坐标
			 *  输出参数
			 *  返回值   	bool
			 */
			bool IsInBound(double dX, double dY) const
			{
				if (dX < _dXmin || dX > _dXmax || dY < _dYmin || dY > _dYmax)
					return false;

				return true;
			}

			bool IsInBound(const PointCloudBoundBox2D& rhs) const
			{
				if (_dXmax >= rhs._dXmax && _dXmin <= rhs._dXmin && _dYmax >= rhs._dYmax &&
					_dYmin <= rhs._dYmin)
					return true;

				return false;
			}
			bool operator==(const PointCloudBoundBox2D& boundBox) const
			{
				return Equivalent(_dXmin, boundBox._dXmin) && Equivalent(_dXmax, boundBox._dXmax) &&
					   Equivalent(_dYmin, boundBox._dYmin) && Equivalent(_dYmax, boundBox._dYmax);
			}

			bool operator!=(const PointCloudBoundBox2D& boundBox) const
			{
				return !Equivalent(_dXmin, boundBox._dXmin) ||
					   !Equivalent(_dXmax, boundBox._dXmax) ||
					   !Equivalent(_dYmin, boundBox._dYmin) || !Equivalent(_dYmax, boundBox._dYmax);
			}
			bool Equivalent(double dNum1, double dNum2) const
			{
				return (fabs(dNum1 - dNum2) < EPSILON);
			}

			double _dXmin; // x最小
			double _dXmax; // x最大
			double _dYmin; // y最小
			double _dYmax; // y最大
		};

		static const double C_COMPARE_PRECISION = 0.01; // 包围盒比较精度
		struct BoundBoxCompare
			: public std::binary_function<PointCloudBoundBox2D, PointCloudBoundBox2D, bool>
		{
			bool operator()(const PointCloudBoundBox2D& lhs, const PointCloudBoundBox2D& rhs) const
			{
				// 比较x最小值
				if (std::fabs(lhs._dXmin - rhs._dXmin) > C_COMPARE_PRECISION)
				{
					if (lhs._dXmin < rhs._dXmin)
						return true;
					if (lhs._dXmin > rhs._dXmin)
						return false;
				}

				// 比较x最大值
				if (std::fabs(lhs._dXmax - rhs._dXmax) > C_COMPARE_PRECISION)
				{
					if (lhs._dXmax < rhs._dXmax)
						return true;
					if (lhs._dXmax > rhs._dXmax)
						return false;
				}

				// 比较y最小值
				if (std::fabs(lhs._dYmin - rhs._dYmin) > C_COMPARE_PRECISION)
				{
					if (lhs._dYmin < rhs._dYmin)
						return true;
					if (lhs._dYmin > rhs._dYmin)
						return false;
				}

				// 比较y最大值
				if (std::fabs(lhs._dYmax - rhs._dYmax) > C_COMPARE_PRECISION)
				{
					if (lhs._dYmax < rhs._dYmax)
						return true;
					if (lhs._dYmax > rhs._dYmax)
						return false;
				}

				return false;
			}
		};

		typedef std::vector<PointCloudBoundBox2D> PointCloudBoundBox2DVector;
		typedef std::map<PointCloudBoundBox2D, int, pc::data::BoundBoxCompare>
			PointCloudBoundToPointNum;

		// 点云类别
		static const char OTHER_CLASSIFY = 0;				  // 其他
		static const char GROUND_CLASSIFY = 1;				  // 地面
		static const char LOW_VEGETATION = 2;				  // 植被
		static const char TOWER_CLASSIFY = 3;				  // 杆塔
		static const char WIRE_CLASSIFY = 4;				  // 导线
		static const char JK_GROUND_WIRE_CLASSIFY = 5;		  // 架空地线
		static const char BUILDING_CLASSIFY = 6;			  // 建筑物
		static const char CROSS_POWER_LINE_CLASSIFY = 7;	  // 被跨越电力线
		static const char ROAD_CLASSIFY = 8;				  // 公路
		static const char RAILWAY_CLASSIFY = 9;				  // 铁路
		static const char RIVERSYSTEM_CLASSIFY = 10;		  // 水系
		static const char JUMPER_WIRE_CLASSIFY = 11;		  // 跳线
		static const char POWER_LINE_CLASSIFY = 12;			  // 电力线
		static const char INSULATOR_CLASSIFY = 13;			  // 绝缘子
		static const char RAILWAY_CARRIERCABLE_CLASSIFY = 14; // 铁路承力索或接触线
		static const char HIGH_VEGETATION = 15;				  // 高植被

		// 自动分类参数
		struct SegmentParam
		{
			CString _strVolLevel;					  // 电压等级
			CString _strTopographicFeatures;		  // 地形特征
			std::vector<osg::Vec3d> _vecTowerPos;	  // 杆塔坐标
			osg::Matrix _vpwMatrix;					  // vpw变换矩阵
			std::vector<osg::Vec3d> _vecSelectPoints; // 框选点坐标
			osg::BoundingBox _boundingBox;	   // 范围包围盒（目前用于单木分割，通过包围盒确定范围）
			std::map<char, bool> _showTypeMap; // 点云分类显示类别
			int _epsg;
			osg::Vec3d _offset_xyz;
			unsigned _nDenoiseLevel;		 // 降噪等级0-5（等级为0或者大于5不降噪）
			std::set<int> _clearTypeCluster; // 清理对应分类所属簇

			SegmentParam() : _nDenoiseLevel(0) {}
			SegmentParam(CString strVolLevel,
						 CString strTopographicFeatures,
						 osg::Matrix matrix,
						 const std::vector<osg::Vec3d>& vecTowerPos,
						 const std::vector<osg::Vec3d>& vecSelectPoints,
						 const std::map<char, bool>& showTypeMap)
				: _strVolLevel(strVolLevel),
				  _strTopographicFeatures(strTopographicFeatures),
				  _vecTowerPos(vecTowerPos),
				  _vpwMatrix(matrix),
				  _vecSelectPoints(vecSelectPoints),
				  _showTypeMap(showTypeMap),
				  _epsg(-1),
				  _nDenoiseLevel(0)
			{
			}
		};

		// 类型重映射
		typedef int (*TypeRemapingFunc)(int, void*, uint32_t nTreeId /*单木分割TreeID*/);

		// 类型可见性
		typedef bool (*ConditionalFunc)(int, void*);

		// 分类进度函数
		typedef int (*SegmentProgressFunc)(double, const char*, void*);

		// 自动分类方法掩码
		enum SegmentMask
		{
			eSegmentGround = MASK(0),		  // 地面分类
			eSegmentPowerCorridors = MASK(1), // 电力线路分类
			eSegmentEnviroments = MASK(2),	  // 环境分类
			eSegmentTrees = MASK(3),		  // 单木分割
			eSegmentTreeSpecies = MASK(4),	  // 树种识别
			eSegmentCrops = MASK(5)			  // 农作物识别
		};

		// 青苗赔偿自动分类设置
		struct CropSegmentSettings
		{
			int numThread;			   // 线程数
			unsigned int segmentMask;  // 分类类型掩码
			unsigned int denoiseLevel; // 降噪等级
			CString strVoltageName;	   // 电压等级名称
			CString strGndFeatureName; // 地形特征名称
			CString strCacheDir;	   // 缓存目录

			SegmentProgressFunc progressCallback; // 进度回调
			TypeRemapingFunc typeRemaping;		  // 类型重映射函数
			ConditionalFunc typeVisible;		  // 类型可见性函数
			ConditionalFunc typeNeedClustering;	  // 类型是否需要进行聚类
			void* progressParam;				  // 进度参数
			void* typeRemapParam;				  // 类型重映射参数
			void* typeVisibleParam;				  // 类型可见性参数
			void* typeClusteringParam;			  // 类型聚类参数

			osg::Matrix cameraVPW;								// 相机到屏幕变换矩阵
			std::vector<std::string> images;					// 影像识别文件列表
			std::vector<std::vector<osg::Vec2d>> pixelPolygons; // 框选多边形列表
			std::vector<osg::Vec3d> towerPositions;				// 预设杆塔坐标
		};

		// 点的索引
		struct tagPointIndex
		{
			tagPointIndex() : _fDistance(0), _nType(0) {};

			tagPointIndex(const osg::Vec3& pnt, float fDistance, uint32_t nType)
				: _pnt(pnt), _fDistance(fDistance), _nType(nType) {};

			osg::Vec3 _pnt;	  // 点
			float _fDistance; // 与目标的距离（平方距离）
			uint32_t _nType;  // 分类类型
		};

		// 渲染模式
		enum ERenderMode
		{
			eTrueColorShading = 0,		// 真彩着色
			eElevationShading = 1,		// 高程着色
			eCategoryShading = 2,		// 类别着色
			eTreeIndividualShading = 3, // 单木渲染
			eDiscardShading = 4,		// 隐藏点云着色
			eRenderModeCount			// 着色模式的数量
		};

		// 点云编辑参数
		struct SEditParam
		{
			SEditParam() {}
			SEditParam(const SEditParam& rhs)
				: _matVPW(rhs._matVPW),
				  _matView(rhs._matView),
				  _fineVPW(rhs._fineVPW),
				  _sliceMatVPW(rhs._sliceMatVPW),
				  _polygonPoints(rhs._polygonPoints),
				  _slicePolygonPoints(rhs._slicePolygonPoints)
			{
			}

			osg::Matrix _matVPW;						 // 视口vpw矩阵
			osg::Matrix _matView;						 // 视口v矩阵
			osg::Matrix _fineVPW;						 // 精细vpw矩阵（用于多边形边缘精细化）
			osg::Matrix _sliceMatVPW;					 // 剖切视口vpw矩阵
			std::vector<osg::Vec3d> _polygonRealPoints;	 // 多边形点集
			std::vector<osg::Vec3d> _polygonPoints;		 // 多边形点集
			std::vector<osg::Vec3d> _slicePolygonPoints; // 剖切视口多边形点集
		};
		struct SCircularEditParam
		{
			SCircularEditParam() : _dR(0.0) {}

			osg::Vec3d _center; // 圆心
			double _dR;			// 半径
		};

		enum ETravelMode
		{
			eFastWideTravel,	// 快速的粗略层级遍历
			eShowContentTravel, // 当前显示内容遍历
			eAllDataTravel,		// 所有数据内容遍历
		};

		struct tagTravelInfo
		{
			tagTravelInfo()
				: fRadius(0.0),
				  bOnlyXYRadius(false),
				  eMode(ETravelMode::eFastWideTravel),
				  bOnlyInLine(false)
			{
			}
			osg::Vec3 beginPnt;
			osg::Vec3 endPnt;
			float fRadius;		// 搜索半径
			bool bOnlyXYRadius; // 搜索半径忽略Z值
			ETravelMode eMode;	// 遍历模式
			bool bOnlyInLine;	// 点仅投影在线上
		};

		struct MeasureResult
		{
			osg::Vec3d minContrastPt;  // 最短距离参考点
			osg::Vec3d minReferencePt; // 最短距离比较点
		};

		enum EMeasureType
		{
			eHorizontalMeasure = 0, // 水平距离（投影到平面上的距离）
			eVerticalMeasure = 1,	// 垂直距离（高度差值）
			eClearanceMeasure = 2,	// 净空距离（两点的距离）
		};

		const int g_nHighlightMinPointSize = 8;
		// 点高亮颜色（洋红色）
		const osg::Vec4 g_HighlightColor(1, 0, 1, 1);

		// 多边形参数
		struct SPolygonParam
		{
			SPolygonParam() : _bAllTraversal(false) {}
			SPolygonParam(const pc::data::SPolygonParam& param)
				: _matVPW(param._matVPW),
				  _sliceMatVPW(param._sliceMatVPW),
				  _polygonPoints(param._polygonPoints),
				  _slicePolygonPoints(param._slicePolygonPoints),
				  _bAllTraversal(param._bAllTraversal)
			{
			}

			osg::Matrix _matVPW;						 // 操作视口vpw矩阵
			osg::Matrix _sliceMatVPW;					 // 剖切视口vpw矩阵
			std::vector<osg::Vec3d> _polygonPoints;		 // 操作视口多边形点集
			std::vector<osg::Vec3d> _slicePolygonPoints; // 剖切视口多边形点集
			bool _bAllTraversal;						 // 全遍历
		};

		// 截面信息
		struct tagPieceinfo
		{
			std::vector<osg::Vec3d> vecPWirePnt; // 当前截面导线点
			std::vector<osg::Vec3d> vecGWirePnt; // 当前截面地线点
			std::vector<osg::Vec3d> vecPlacePnt; // 当前截面地面截面线
			osg::Vec3d sectionPnt;				 // 截面点（杆塔起终连线等距截取点）
		};
		// 防雷提取参数
		struct SLightningParam
		{
			SLightningParam()
				: _nPiece(0),
				  _dLineWidth(0.0),
				  _dBeginAngle(0.0),
				  _dEndAngle(0.0),
				  _dSectionWidth(0.0),
				  _nWireType(0),
				  _nGWireType(0),
				  _nGroundType(0),
				  _nJumpType(0),
				  _nWaterSysType(0)
			{
			}
			unsigned _nPiece;		 // 截面数量
			osg::Vec3d _beginPnt;	 // 起点
			osg::Vec3d _endPnt;		 // 终点
			double _dLineWidth;		 // 线路宽度
			double _dBeginAngle;	 // 起点旋转角度
			double _dEndAngle;		 // 终点旋转角度
			double _dSectionWidth;	 // 截面宽度
			unsigned _nWireType;	 // 导线类型
			unsigned _nGWireType;	 // 地线类型
			unsigned _nGroundType;	 // 地面类型
			unsigned _nJumpType;	 // 跳线类型
			unsigned _nWaterSysType; // 水系类型
		};

		// 降噪参数
		struct SDenoiseParam
		{
			double _dRadius;	  // 检测范围
			unsigned _nMinPtSize; // 最小点数
		};
		struct SDenoiseCfg
		{
			std::vector<SDenoiseParam> params; // 降噪参数列表
			std::set<int> notDenoiseTypes;	   // 不参与降噪的类型
		};

		// 点云信息
		struct SPcInfo
		{
			SPcInfo() : dMinZ(DBL_MAX), dMaxZ(-DBL_MAX) {}
			double dMinZ;					 // 最小高程
			double dMaxZ;					 // 最大高程
			std::vector<osg::Vec3d> _points; // 点坐标
		};

		// 点关联信息
		struct PointRelateInfo
		{
			PointRelateInfo()
			{
				type = 0;
				pointIndex = -1;
			}
			osg::Vec3d point;  // 点坐标
			unsigned type;	   // 点类型
			CString pageLodId; // 点所在pageLodId
			size_t pointIndex; // 点在pageLod中的点索引
		};

		// page层级
		enum EPageLodLevel
		{
			eRough,	   // 粗糙
			eSubRough, // 次粗糙
			eSubFine,  // 次精细
			eFine,	   // 精细
			eAll,	   // 所有层级
		};

		// 树木信息
		struct treeInfo
		{
			treeInfo() : dValidSweepHeight(0.0) {}
			osg::Vec3 topPnt;		  // 顶点
			osg::Vec3 bottomPnt;	  // 底点
			double dValidSweepHeight; // 有效树木扫高
									  /*
										  》树顶底位置P、Q；树木有效扫角高度H
										  》树木宽度大于 系数L(L>0) 倍高度，则H等于PQ距离
										  》树顶到 系数M(0<M<1) 处点个数大于 系数N(0<N<1)
										 倍总点数，则H等于包围盒对角长度。反之等于PQ距离
									  */
		};
		const CString gl_strTreeInfoCoefficientCfg =
			L"..\\PluginsConfig\\PointCloud\\树木信息系数.xml";

		// 分类操作后的簇
		struct SClusterAfterClassify
		{
			typedef std::map<int, int> ClusterToType; // <簇id对应，分类类型>
			ClusterToType newClusters;				  // 新增的簇
			ClusterToType removeClusters;			  // 删除的簇
			ClusterToType modifyClusters;			  // 修改的簇

			std::map<int, osg::BoundingBox> boundPerCluster; // 类簇的包围盒
			std::map<int, std::vector<std::vector<osg::Vec3d>>>
				polygonPerCluster; // 类簇的多边形轮廓
			std::map<int, int>
				_mapClusterOldToNewIds; // 老簇与新簇的id映射关系,key为新簇value为旧簇
		};


		// 青苗分类参数
		struct SQMClassifyParam
		{
			SQMClassifyParam()
				: _nSegmentIndex(0),
				  _bCoverCluster(false),
				  _bNeedCreateCluster(false),
				  _bOnlyOneCluster(false),
				  _forceWaitTask(false),
				  _bMoifySpecificType(false)
			{
			}
			int _nSegmentIndex;		  // 分类索引
			bool _bCoverCluster;	  // 如果范围存在簇，是否覆盖（覆盖指重置相交区域点的簇id）
			bool _bNeedCreateCluster; // 是否需要将框选点创建为簇
			bool
				_bOnlyOneCluster; // 分类类型创建的簇是否需要唯一（需要唯一当存在改类型的簇时，后续创建仅追加）

			std::vector<osg::Vec3d> _spanPolygon;
			bool _forceWaitTask; // 强制等待后台任务执行完毕

			bool _bMoifySpecificType;	  // 是否仅修改特定类型
			std::set<int> _specificTypes; // 特定类型
		};

		// 档位信息
		struct SSpanParam
		{
			SSpanParam() : _dBeginAngle(0.0), _dEndAngle(0.0) {}
			osg::Vec3d _beginPnt;				 // 起点
			osg::Vec3d _endPnt;					 // 终点
			double _dBeginAngle;				 // 起点转角
			double _dEndAngle;					 // 终点转角
			std::vector<osg::Vec3d> _spanPolygn; // 档多边形
		};

		// 水系自动填充参数
		struct SWaterSystemAutoFillingParam
		{
			SWaterSystemAutoFillingParam() : _dMinArea(0.0), _dDensity(0.0), _nWaterSystemType(0) {}
			bool IsValid() const
			{
				return (_towerPnts.size() == _angleBisectionDis.size() && _towerPnts.size() >= 2);
			}

			std::vector<osg::Vec3d> _towerPnts;			// 杆塔点
			std::vector<osg::Vec3d> _angleBisectionDis; // 杆塔点角平分线方向
			double _dMinArea;							// 限制面积
			double _dDensity;							// 填充密度
			int _nWaterSystemType;						// 水系类型索引
			std::set<int> _removeTypes;					// 剔除干扰孔洞的类型（导地线）
			osg::Vec4 _color;							// 真实颜色
		};

		// 植被自动分类参数
		struct SVegetationAutoClassifyParam
		{
			SVegetationAutoClassifyParam()
				: _dShrubMinHeight(0.0),
				  _dShrubMaxHeight(0.0),
				  _nVegetationType(0),
				  _nGrassType(0),
				  _nShrubType(0),
				  _nForestType(0),
				  _nGoundType(0)
			{
			}
			bool IsValid() const
			{
				return (_towerPnts.size() == _angleBisectionDis.size() && _towerPnts.size() >= 2);
			}

			std::vector<osg::Vec3d> _towerPnts;			// 杆塔点
			std::vector<osg::Vec3d> _angleBisectionDis; // 杆塔点角平分线方向
			double _dShrubMinHeight;					// 灌木最小高度
			double _dShrubMaxHeight;					// 灌木最大高度
			int _nVegetationType;						// 植被类型
			int _nGrassType;							// 草地类型
			int _nShrubType;							//  灌木类型
			int _nForestType;							// 林木类型
			int _nGoundType;							// 地面类型
		};
	}
}

#define INIT_CROP_SEGMENT_SETTINGS(s)                                                              \
	do                                                                                             \
	{                                                                                              \
		(s).numThread = -1;                                                                        \
		(s).segmentMask = pc::data::eSegmentGround;                                                \
		(s).denoiseLevel = 3;                                                                      \
		(s).typeRemaping = NULL;                                                                   \
		(s).typeVisible = NULL;                                                                    \
		(s).progressCallback = NULL;                                                               \
		(s).typeRemapParam = NULL;                                                                 \
		(s).typeVisibleParam = NULL;                                                               \
		(s).progressParam = NULL;                                                                  \
	} while (0);


#define PC_ERROR(...) d3s::CLog::Log(d3s::CLog::err, __VA_ARGS__)
#define PC_WARN(...) d3s::CLog::Log(d3s::CLog::warn, __VA_ARGS__)
#define PC_INFO(...) d3s::CLog::Log(d3s::CLog::info, __VA_ARGS__)
#define PC_DEBUG(...) d3s::CLog::Log(d3s::CLog::debug, __VA_ARGS__)


#endif // POINTCLOUDMANAGERDEF_H_