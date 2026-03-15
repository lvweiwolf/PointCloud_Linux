#ifndef API_H_
#define API_H_

#include <src/core/pointTypes.h>


namespace d3s {
	namespace pcs {

		/**
		 *  @brief    读取点云
		 *
		 *  @param    const std::string & filename					LAS点云文件路径
		 *  @param    PointCloudView<PointPCLH> & pcv				点云数据
		 *  @param    bool voxelSample								是否进行抽稀
		 *  @param    double voxelSize								体素大小
		 *
		 *  @return   bool											是否读取成功
		 */
		bool readPointCloud(const std::string& filename,
							PointCloudView<PointPCLH>& pcv,
							bool voxelSample = false,
							double voxelSize = 0.1);

		bool readPointCloud2(const std::string& filename, PointCloudView<PointPCLH>& pcv);

		/**
		 *  @brief    写入点云
		 *
		 *  @prarm	 const std::string & filename					写入点云文件路径
		 *  @prarm	 const PointCloudView<PointPCLH> & pcv			点云数据
		 *
		 *  @return    bool
		 */
		bool writePointCloud(const std::string& filename, const PointCloudView<PointPCLH>& pcv);
		bool writePointCloud(const std::string& filename,
							 const PointCloudView<PointPCLH>& pcv,
							 const std::vector<int>& indices);



		/**
		 *  @brief    点云降噪，KNN统计分析
		 *
		 *  @prarm	 PointCloudView<PointPCLH> & pcv				点云数据
		 *  @prarm	 int meanK										邻近数量
		 *  @prarm	 double stdMul									标准差阈值
		 *
		 *  @return    void
		 */
		void denoiseStatistical(PointCloudView<PointPCLH>& pcv, int meanK, double stdMul);

		void denoiseStatistical(PointCloudView<PointPCLH>& pcv,
								const std::vector<int>& indices,
								int meanK,
								double stdMul,
								std::vector<int>& inliners,
								std::vector<int>& outliers);

		/**
		 *  @brief    点云降噪，半径内最小点数法
		 *
		 *  @param    PointCloudView<PointPCLH> & pcv
		 *  @param    double radius
		 *  @param    int minpts
		 *
		 *  @return    void
		 */
		void denoiseRadius(PointCloudView<PointPCLH>& pcv, double radius, int minpts);


		/**
		 *  @brief    点云降噪，Z值范围
		 *
		 *  @prarm	 PointCloudView<PointPCLH> & pcv
		 *  @prarm	 const std::string& fieldname
		 *  @prarm	 double low
		 *  @prarm	 double high
		 *
		 *  @return    void
		 */
		void denoiseRange(PointCloudView<PointPCLH>& pcv,
						  const std::string& fieldname,
						  double low,
						  double high);

		/**
		 *  @brief    设置点云类别
		 *
		 *  @param    ClassificationType label						类别
		 *  @param    PointCloudView<PointPCLH> & pcv				点云数据
		 *
		 *  @return    void
		 */
		void setClassification(ClassificationType label, PointCloudView<PointPCLH>& pcv);


		/**
		 *  @brief    设置点云类别
		 *
		 *  @param    ClassificationType label						类别
		 *  @param    const std::vector<int> & indices				点云索引
		 *  @param    PointCloudView<PointPCLH> & pcv				点云数据
		 *
		 *  @return    void
		 */
		void setClassification(ClassificationType label,
							   const std::vector<int>& indices,
							   PointCloudView<PointPCLH>& pcv);


		/**
		 *  @brief    获得所属类别的点云
		 *
		 *  @prarm	 const PointCloudView<PointPCLH> & pcv			点云数据
		 *  @prarm	 ClassificationType label						类别
		 *  @prarm	 std::vector<int> & indices						点云索引
		 *
		 *  @return    void
		 */
		void getClassification(const PointCloudView<PointPCLH>& pcv,
							   ClassificationType label,
							   std::vector<int>& indices);

		/**
		 *  @brief    获得在指定范围内，所属类别的点云
		 *
		 *  @param    const PointCloudView<PointPCLH> & pcv			点云数据
		 *  @param    double xmin									下界x
		 *  @param    double ymin									下界y
		 *  @param    double xmax									上界x
		 *  @param    double ymax									上界y
		 *  @param    ClassificationType label						类别
		 *  @param    std::vector<int> & indices					点云索引
		 *
		 *  @return    void
		 */
		void getClassification(const PointCloudView<PointPCLH>& pcv,
							   double xmin,
							   double ymin,
							   double xmax,
							   double ymax,
							   ClassificationType label,
							   std::vector<int>& indices);

	}
}


#endif // API_H_