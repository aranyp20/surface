#include "curvaturecalculator.h"

namespace core
{
    // TODO before integrated: cps.P size check + origo center precondition
    std::vector<CurvatureCalculator::SurfaceParam> CurvatureCalculator::InputPoints::calcUVs() const
    {
        std::vector<SurfaceParam> retval;

        // retval.push_back({0, 0});

        std::vector<double> alphas;
        double alpha_sum = 0;
        for (size_t i = 0; i < P.size(); i++)
        {
            const auto &p_i = P[i];
            const auto &p_i_next = i == P.size() - 1 ? P[0] : P[i + 1];
            const auto alpha = std::acos(p_i.normalized().dot(p_i_next.normalized()));
            alphas.push_back(alpha);
            alpha_sum += alpha;
        }

        const auto alpha_normalizer = 2 * M_PI / alpha_sum;
        for (auto &alpha : alphas)
        {
            alpha *= alpha_normalizer;
        }

        for (size_t i = 0; i < P.size(); i++)
        {
            const auto h_i = P[i].norm();

            double sum_a = 0;
            if (i != 0)
            {
                for (size_t j = 0; j < i; j++)
                {
                    sum_a += alphas[j];
                }
            }

            retval.push_back({h_i * std::cos(sum_a), h_i * std::sin(sum_a)});
        }

        return retval;
    }

    CurvatureCalculator::InputPoints CurvatureCalculator::InputPoints::translatePoints() const
    {
        auto clone = *this;

        for (auto &cp : clone.P)
        {
            cp -= clone.center;
        }
        clone.center = {0, 0, 0};

        return clone;
    }

    CurvatureCalculator::DerResults CurvatureCalculator::calcDer(const InputPoints &ipp) const
    {
        const auto uvs = ipp.calcUVs();

        const auto res_size = 5;
        Eigen::MatrixXd A = Eigen::MatrixXd::Random(uvs.size(), res_size);
        Eigen::MatrixXd c = Eigen::MatrixXd::Random(uvs.size(), 3);

        for (int i = 0; i < uvs.size(); i++)
        {
            const auto &uv = uvs[i];
            A(i, 0) = uv.u;
            A(i, 1) = uv.v;
            A(i, 2) = 0.5 * uv.u * uv.u;
            A(i, 3) = uv.u * uv.v;
            A(i, 4) = 0.5 * uv.v * uv.v;

            c(i, 0) = ipp.P[i][0];
            c(i, 1) = ipp.P[i][1];
            c(i, 2) = ipp.P[i][2];
        }

        Eigen::MatrixXd b = Eigen::MatrixXd::Random(res_size, 3);

        if (uvs.size() >= 5)
        {
            // Eigen uses the least squares method for overdetermined systems by default
            b = A.colPivHouseholderQr().solve(c);
        }
        else
        {

            // Min norm methods
            b = A.transpose() * (A * A.transpose()).inverse() * c;
        }

        return {b.row(0), b.row(1), b.row(2), b.row(3), b.row(4)};
    }

    Eigen::Vector3d CurvatureCalculator::S(const double u, const double v, const DerResults &Ss) const
    {
        const Eigen::Vector3d S00{0, 0, 0};

        return S00 + u * Ss.Su + v * Ss.Sv + 0.5 * u * u * Ss.Suu + u * v * Ss.Suv + 0.5 * v * v * Ss.Svv;
    }

    void CurvatureCalculator::calcCurvature(const InputPoints &ipp)
    {
        const auto ip = ipp.translatePoints();

        const auto eq = calcDer(ip);

        fundamental_elements.E = eq.Su.norm() * eq.Su.norm();
        fundamental_elements.G = eq.Sv.norm() * eq.Sv.norm();
        fundamental_elements.F = eq.Su.dot(eq.Sv);

        normal = eq.Su.cross(eq.Sv).normalized();
        
        fundamental_elements.L = eq.Suu.dot(normal);
        fundamental_elements.M = eq.Suv.dot(normal);
        fundamental_elements.N = eq.Svv.dot(normal);

    }

    double CurvatureCalculator::getCurvature() const
    {
        const auto& fe = fundamental_elements;
        return (fe.L * fe.G - fe.M * fe.F + fe.N * fe.E - fe.M * fe.F) / (2 * fe.E * fe.G - fe.F * fe.F);
    }

    double CurvatureCalculator::getGaussianCurvature() const
    {
        const auto& fe = fundamental_elements;
        return (fe.N * fe.E - 2 * fe.M * fe.F + fe.L * fe.G) / (fe.E * fe.G - fe.F * fe.F);
    }

    double CurvatureCalculator::getMaxPrincipleCurvature() const
    {
        const auto main_curvature = getCurvature();
        const auto gaussian_curvature = getGaussianCurvature();
        return 0; 
    }

/*
    k2 = B - k1
    k1(B - k1) = A

    k1*B - k1*k1 -A= 0

    a=-1
    b=B
    c=-A
*/

/*
    void CurvatureCalculator::calcCurvatures(common::MyMesh &mesh) const
    {


        OpenMesh::VPropHandleT<double> doubleValues;
        mesh.add_property(doubleValues, "doubleValues");

        for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
        {


            mesh.property(doubleValues, vh) = calcCurvature(cip);
        }

        for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
        {
            common::MyMesh::VertexHandle vh = *v_it;
            double doubleValue = mesh.property(doubleValues, vh);
            //std::cout << "Vertex " << vh.idx() << ": Double Value = " << doubleValue << std::endl;
        }
    }
*/

    void CurvatureCalculator::execute(common::MyMesh::VertexHandle &vh)
    {
        const common::MyMesh::Point& vertexPos = mesh.point(vh);

        InputPoints cip;
        cip.center = Eigen::Vector3d(vertexPos[0], vertexPos[1], vertexPos[2]);
        for (common::MyMesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(vh); voh_it.is_valid(); ++voh_it)
        {
            common::MyMesh::VertexHandle neighborVh = mesh.to_vertex_handle(*voh_it);

            const auto neighborPos = mesh.point(neighborVh); // TODO ref?
            cip.P.emplace_back(neighborPos[0], neighborPos[1], neighborPos[2]);
        }

        calcCurvature(cip);
    }

    void CurvatureCalculator::execute(const Eigen::Vector3d vertex_pos, const std::vector<Eigen::Vector3d>& neighbors)
    {
        InputPoints cip;
        cip.center = Eigen::Vector3d(vertex_pos[0], vertex_pos[1], vertex_pos[2]);
        for (const auto& neighbor_pos : neighbors)
        {
            cip.P.emplace_back(neighbor_pos[0], neighbor_pos[1], neighbor_pos[2]);
        }

        calcCurvature(cip);
    }


    CurvatureCalculator::FundamentalElements CurvatureCalculator::getFundamentalElements() const
    {
        return fundamental_elements;
    }

    Eigen::Vector3d CurvatureCalculator::getNormal() const
    {
        return normal;
    }

    CurvatureCalculator::CurvatureCalculator(common::MyMesh &mesh) : mesh(mesh)
    {
    }






    /*
        std::vector<Triangle3d> tessellateSurface(const size_t resolution, const DerResults &Ss)
        {

    #define uvBOUND 20

            const double u_min = -uvBOUND;
            const double u_max = uvBOUND;
            const double v_min = -uvBOUND;
            const double v_max = uvBOUND;

            const double u_step_distance = (u_max - u_min) / resolution;
            const double v_step_distance = (v_max - v_min) / resolution;

            std::vector<Eigen::Vector3d> values;
            for (size_t i = 0; i < resolution; i++)
            {
                const double u = u_min + i * u_step_distance;
                for (size_t j = 0; j < resolution; j++)
                {
                    const double v = v_min + j * v_step_distance;
                    values.push_back(S(u, v, Ss));
                }
            }

            std::vector<Triangle3d> retval;

            // TODO refactor

            std::fstream fw("res1.obj", std::ios::out);

            fw << "# Vertices\n";

            for (const auto &vertex : values)
            {
                fw << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
            }

            fw << "\n# Faces\n";

            for (size_t i = 0; i < (resolution - 1) * (resolution - 1); i++)
            {
                const size_t left_right_index = std::floor(i / (resolution - 1)) + i;
                retval.push_back({values[left_right_index], values[left_right_index + resolution], values[left_right_index + resolution + 1]});
                retval.push_back({values[left_right_index], values[left_right_index + resolution + 1], values[left_right_index]}); // TODO check indices

                fw << "f " << left_right_index + 1 << " " << left_right_index + resolution + 1 << " " << left_right_index + resolution + 2 << std::endl;
                fw << "f " << left_right_index + 1 << " " << left_right_index + resolution + 2 << " " << left_right_index + 2 << std::endl;
            }

            return retval;
        }
    */
}
