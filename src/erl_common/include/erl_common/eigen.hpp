#pragma once
// disable warning when using Eigen
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "logging.hpp"

#include <fstream>

#include <unsupported/Eigen/CXX11/Tensor>

// https://stackoverflow.com/questions/4433950/overriding-functions-from-dynamic-libraries
// https://danieldk.eu/Posts/2020-08-31-MKL-Zen.html
extern "C" {
[[maybe_unused]] int
mkl_serv_intel_cpu_true();
}

namespace Eigen {
    // MATRIX
    using MatrixXl = MatrixX<long>;
    using MatrixXb = MatrixX<bool>;
    using MatrixX8U = MatrixX<uint8_t>;
    using Matrix2Xl = Matrix<long, 2, Eigen::Dynamic>;
    using Matrix3Xl = Matrix<long, 3, Eigen::Dynamic>;
    using Matrix23d = Matrix<double, 2, 3>;
    using Matrix24d = Matrix<double, 2, 4>;

    template<typename T, int rows, int cols>
    using RMatrix = Matrix<T, rows, cols, RowMajor>;

    using RMatrix23d = RMatrix<double, 2, 3>;
    using RMatrix2Xd = RMatrix<double, 2, Eigen::Dynamic>;

    template<typename T>
    using Scalar = Matrix<T, 1, 1>;
    using Scalari = Scalar<int>;
    using Scalard = Scalar<double>;

    // VECTOR
    using VectorXl = VectorX<long>;
    using VectorXb = VectorX<bool>;
    using VectorX8U = VectorX<uint8_t>;
}  // namespace Eigen

namespace erl::common {

    // https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
    enum class EigenTextFormat {
        kDefaultFmt = 0,
        kCommaInitFmt = 1,
        kCleanFmt = 2,
        kOctaveFmt = 3,
        kNumpyFmt = 4,
        kCsvFmt = 5,
    };

    Eigen::IOFormat
    GetEigenTextFormat(EigenTextFormat format);

    template<typename T>
    void
    SaveEigenMatrixToTextFile(
        const std::string &file_path,
        const Eigen::Ref<const Eigen::MatrixX<T>> &matrix,
        const EigenTextFormat format = EigenTextFormat::kDefaultFmt) {

        std::ofstream ofs(file_path);
        if (!ofs.is_open()) { throw std::runtime_error("Could not open file " + file_path); }
        ofs << matrix.format(GetEigenTextFormat(format));
        ofs.close();
    }

    template<typename T, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic, int RowMajor = Eigen::ColMajor>
    Eigen::Matrix<T, Rows, Cols, RowMajor>
    LoadEigenMatrixFromTextFile(const std::string &file_path, const EigenTextFormat format = EigenTextFormat::kDefaultFmt) {
        std::vector<T> data;
        std::ifstream ifs(file_path);

        if (!ifs.is_open()) { throw std::runtime_error("Could not open file " + file_path); }

        std::string row_string;
        std::string entry_string;
        int cols = 0;
        int rows = 0;

        char delim = ',';
        switch (format) {
            case EigenTextFormat::kDefaultFmt:
                delim = ' ';
                break;
            case EigenTextFormat::kCommaInitFmt:
            case EigenTextFormat::kCleanFmt:
            case EigenTextFormat::kOctaveFmt:
            case EigenTextFormat::kNumpyFmt:
            case EigenTextFormat::kCsvFmt:
                delim = ',';
                break;
        }

        while (std::getline(ifs, row_string)) {
            std::stringstream row_stream(row_string);
            int row_cols = 0;
            while (std::getline(row_stream, entry_string, delim)) {
                if (entry_string.empty()) { continue; }
                if (entry_string == ";" || entry_string == "]" || entry_string == "[") { continue; }
                data.push_back(T(std::stod(entry_string)));
                if (rows == 0) {
                    cols++;
                } else {
                    row_cols++;
                }
            }
            if (rows == 0) {
                ERL_ASSERTM(
                    Cols == Eigen::Dynamic || cols == Cols,
                    "Number of columns in file does not match template parameter. Expected {}, got {}",
                    Cols,
                    cols);
            } else {
                ERL_ASSERTM(cols == row_cols, "Invalid matrix file: row {} has {} columns, expected {}", rows, row_cols, cols);
            }
            rows++;
        }
        ifs.close();

        ERL_ASSERTM(Rows == Eigen::Dynamic || rows == Rows, "Number of rows in file does not match template parameter. Expected {}, got {}", Rows, rows);

        if (RowMajor == Eigen::RowMajor) {
            Eigen::MatrixX<T> matrix(rows, cols);
            std::copy(data.begin(), data.end(), matrix.data());
            return matrix;
        }
        Eigen::MatrixX<T> matrix(cols, rows);
        std::copy(data.begin(), data.end(), matrix.data());
        return matrix.transpose();
    }

    template<typename T>
    void
    SaveEigenMatrixToBinaryFile(const std::string &file_path, const Eigen::Ref<const Eigen::MatrixX<T>> &matrix) {
        const Eigen::MatrixX<T> matrix_copy = matrix;  // make a copy to make sure that the memory is contiguous
        std::ofstream ofs(file_path, std::ios::binary);
        if (!ofs.is_open()) { throw std::runtime_error("Could not open file " + file_path); }
        const long matrix_shape[2] = {matrix_copy.rows(), matrix_copy.cols()};
        ofs.write(reinterpret_cast<const char *>(matrix_shape), 2 * sizeof(long));
        ofs.write(reinterpret_cast<const char *>(matrix_copy.data()), matrix_copy.size() * sizeof(T));
        ofs.close();
    }

    template<typename T, int Rows, int Cols>
    Eigen::Matrix<T, Rows, Cols>
    LoadEigenMatrixFromBinaryFile(const std::string &file_path) {
        std::ifstream ifs(file_path, std::ios::binary);
        if (!ifs.is_open()) { throw std::runtime_error("Could not open file " + file_path); }

        ifs.seekg(0, std::ios::end);
        long file_size = ifs.tellg();
        ifs.seekg(0, std::ios::beg);
        ERL_ASSERTM(
            (file_size - 2 * sizeof(long)) % sizeof(T) == 0,
            "File size is not a multiple of the element size. File size: {}, element size: {}",
            file_size,
            sizeof(T));
        long matrix_shape[2];
        ifs.read(reinterpret_cast<char *>(matrix_shape), 2 * sizeof(long));

        ERL_ASSERTM(
            Rows == Eigen::Dynamic || matrix_shape[0] == Rows,
            "Number of rows in file does not match template parameter. Expected {}, got {}",
            Rows,
            matrix_shape[0]);
        ERL_ASSERTM(
            Cols == Eigen::Dynamic || matrix_shape[1] == Cols,
            "Number of columns in file does not match template parameter. Expected {}, got {}",
            Cols,
            matrix_shape[1]);

        long num_elements = (file_size - 2 * sizeof(long)) / sizeof(T);
        ERL_ASSERTM(
            num_elements == matrix_shape[0] * matrix_shape[1],
            "Broken matrix file: expected {} elements, got {}",
            matrix_shape[0] * matrix_shape[1],
            num_elements);

        Eigen::MatrixX<T> matrix(matrix_shape[0], matrix_shape[1]);
        ifs.read(reinterpret_cast<char *>(matrix.data()), static_cast<long>(num_elements * sizeof(T)));
        ifs.close();
        return matrix;
    }

    template<EigenTextFormat Format, typename Matrix>
    std::string
    EigenToString(const Matrix &matrix) {
        std::stringstream ss;
        ss << matrix.format(GetEigenTextFormat(Format));
        return ss.str();
    }

    template<typename Matrix>
    std::string
    EigenToDefaultFmtString(const Matrix &matrix) {
        std::stringstream ss;
        ss << matrix.format(GetEigenTextFormat(EigenTextFormat::kDefaultFmt));
        return ss.str();
    }

    template<typename Matrix>
    std::string
    EigenToCommaInitFmtString(const Matrix &matrix) {
        std::stringstream ss;
        ss << matrix.format(GetEigenTextFormat(EigenTextFormat::kCommaInitFmt));
        return ss.str();
    }

    template<typename Matrix>
    std::string
    EigenToCleanFmtString(const Matrix &matrix) {
        std::stringstream ss;
        ss << matrix.format(GetEigenTextFormat(EigenTextFormat::kCleanFmt));
        return ss.str();
    }

    template<typename Matrix>
    std::string
    EigenToOctaveFmtString(const Matrix &matrix) {
        std::stringstream ss;
        ss << matrix.format(GetEigenTextFormat(EigenTextFormat::kOctaveFmt));
        return ss.str();
    }

    template<typename Matrix>
    std::string
    EigenToNumPyFmtString(const Matrix &matrix) {
        std::stringstream ss;
        ss << matrix.format(GetEigenTextFormat(EigenTextFormat::kNumpyFmt));
        return ss.str();
    }

    template<typename Matrix>
    std::string
    EigenToCsvFmtString(const Matrix &matrix) {
        std::stringstream ss;
        ss << matrix.format(GetEigenTextFormat(EigenTextFormat::kCsvFmt));
        return ss.str();
    }

}  // namespace erl::common

#pragma GCC diagnostic pop
