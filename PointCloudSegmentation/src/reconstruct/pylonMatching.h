//////////////////////////////////////////////////////////////////////
// 文件名称：pylonReconstruct.h
// 功能描述：杆塔重建算法
// 创建标识：吕伟	2022/12/21
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once
#include <string>
#include <vector>
#include <osg/BoundingBox>
#include <osg/Matrix>
#include "../core/pointTypes.h"

namespace d3s {
	namespace pcs {

		struct PylonMatchingOptions
		{
			double rms_threshold;
			double bbox_threshold;
			int num_registration;
		};


		class PylonMatching
		{
		public:
			PylonMatching(const PylonMatchingOptions& options,
						  PointCloudViewPtr input,
						  osg::Matrix transform);

			~PylonMatching();

			/**
			*  @brief    设置匹配库路径
			*
			*  @prarm	 const std::string & libpath
			*
			*  @return   void
			*/
			void SetMatchingLibrary(const std::string& libpath);

			/**
			*  @brief    获得匹配结果
			*
			*  @return   std::string
			*/
			std::vector<std::string> GetMatchingResult() const;

			/**
			*  @brief    铁塔组件模型匹配
			*
			*  @return   void
			*/
			void Matching();

		private:
			/**
			*  @brief    计算对齐到X轴后的点云包围盒
			*
			*  @return   osg::BoundingBox
			*/
			osg::BoundingBox ComputeAlignedBBox();

			/**
			*  @brief    包围盒是否相似
			*
			 *  @prarm	 const osg::BoundingBox & lhs			包围盒1
			 *  @prarm	 const osg::BoundingBox & rhs			包围盒2
			*
			*  @return   bool
			*/
			bool Similarity(const osg::BoundingBox& lhs, const osg::BoundingBox& rhs);

			/**
			*  @brief    配准，并计算 RMS
			*
			*  @prarm	 PointCloudView<PointPCLH> & source		配准点云
			*  @prarm	 PointCloudView<PointPCLH> & target		目标点云
			*  @prarm	 osg::Matrix & transform				变换矩阵
			*
			*  @return   double
			*/
			double RegistrationRMS(PointCloudView<PointPCLH>& source,
								   PointCloudView<PointPCLH>& target,
								   osg::Matrix& transform);

		private:
			PylonMatchingOptions _options;
			PointCloudViewPtr _input;

			osg::Matrix _transform;
			std::string _library;
			
			std::vector<std::string> _result;
		};
	}
}