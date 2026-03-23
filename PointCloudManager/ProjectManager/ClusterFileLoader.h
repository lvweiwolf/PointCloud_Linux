#ifndef CLUSTER_FILE_LOADER_H_
#define CLUSTER_FILE_LOADER_H_

#include <include/cstring.h>
#include <include/ReferenceCountObj.h>

class CClusterFileLoader : public d3s::ReferenceCountObj
{
public:
	/**
	 * @brief 加载文件
	 * @param strProjectID 项目ID
	 * @param strFileName 文件名称
	 * @return void
	 */
	void LoadFile(const CString& strProjectID, const CString& strFileName);

	/**
	 * @brief 保存文件
	 * @param strProjectID 项目ID
	 * @param strFileName 文件名称
	 * @return void
	 */
	void SaveFile(const CString& strProjectID, const CString& strFileName);

	/**
	 * @brief 关闭文件
	 * @param strProjectid 项目ID
	 * @return void
	 */
	void CloseFile(const CString& strProjectid);
};


#endif // CLUSTER_FILE_LOADER_H_