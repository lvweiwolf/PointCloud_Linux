//////////////////////////////////////////////////////////////////////
// 文件名称：timer.h
// 功能描述：性能测试计时工具
// 创建标识：吕伟	2021/2/5
// 修改标识：
// 修改描述：
// 文件版权：江西博微新技术有限公司
//////////////////////////////////////////////////////////////////////
#pragma once

#include <chrono>

namespace d3s {
	namespace pcs {

		class Timer
		{
		public:
			Timer();

			/**
			 *  @brief    计时器状态控制
			 *
			 */
			void Start();
			void Restart();
			void Pause();
			void Resume();
			void Reset();

			/**
			 *  @brief    计时器数值单位转换
			 *
			 *  @return   double
			 */
			double ElapsedMicroSeconds() const;
			double ElapsedSeconds() const;
			double ElapsedMinutes() const;
			double ElapsedHours() const;

			/**
			 *  @brief    按照单位打印
			 */
			/*void PrintSeconds() const;
			void PrintMinutes() const;
			void PrintHours() const;*/

		protected:
			bool _started;
			bool _paused;

			std::chrono::high_resolution_clock::time_point _start_time;
			std::chrono::high_resolution_clock::time_point _pause_time;
		};
	}
}