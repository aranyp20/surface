#include <GLFW/glfw3.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>


struct DerResults
{
    Eigen::Vector3d Su{0,0,0};
    Eigen::Vector3d Sv{0,0,0};
    Eigen::Vector3d Suu{0,0,0};
    Eigen::Vector3d Suv{0,0,0};
    Eigen::Vector3d Svv{0,0,0};
};

struct SurfaceParam
{
    double u = 0;
    double v = 0;
};

//TODO u,v should be here
struct InputPoints
{
    Eigen::Vector3d center;
    std::vector<Eigen::Vector3d> P;

    void translatePoints()
    {
        for (auto& cp : P)
        {
            cp -= center;
        }
        center = {0,0,0};
    }
};


//TODO before integrated: cps.P size check + origo center precondition
std::vector<SurfaceParam> calcUVs(const InputPoints& cps)
{
    std::vector<SurfaceParam> retval;

    retval.push_back({0,0});

    std::vector<double> alphas;
    double alpha_sum = 0;
    for(size_t i = 0; i < cps.P.size(); i++)
    {
        const auto& p_i = cps.P[i];
        const auto& p_i_next = i == cps.P.size() - 1 ? cps.P[0] : cps.P[i+1];
        const auto alpha = std::acos(p_i.normalized().dot(p_i_next.normalized()));
        alphas.push_back(alpha);
        alpha_sum += alpha;
    }
    
    const auto alpha_normalizer = 2 * M_PI / alpha_sum;
    for(auto& alpha : alphas)
    {
        alpha *= alpha_normalizer;
    }

    for(size_t i = 0; i < cps.P.size(); i++)
    {
        const auto h_i = cps.P[i].norm();

        double sum_a = 0;
        if (i != 0){
            for(size_t j = 0; j < i; j++)
            {
                sum_a += alphas[j];
            }
        }

        retval.push_back({h_i * std::cos(sum_a), h_i * std::sin(sum_a)});
    }




    return retval;
}


DerResults calcDer(const InputPoints& ipp, const std::vector<SurfaceParam>& uvs)
{
    const auto res_size = 5;
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(uvs.size(), res_size);
    Eigen::VectorXd c_x = Eigen::VectorXd::Random(uvs.size());
    Eigen::VectorXd c_y = Eigen::VectorXd::Random(uvs.size());
    Eigen::VectorXd c_z = Eigen::VectorXd::Random(uvs.size());


    for(int i=0;i<uvs.size();i++){
        const auto& uv = uvs[i];
        A(i,0) = uv.u;
        A(i,1) = uv.v;
        A(i,2) = 0.5 * uv.u * uv.u;
        A(i,3) = uv.u * uv.v;
        A(i,4) = 0.5 * uv.v * uv.v;

        if (i==0) {
            c_x(i) = ipp.center[0];
            c_y(i) = ipp.center[1];
            c_z(i) = ipp.center[2];
        }
        else {
            c_x(i) = ipp.P[i-1][0];
            c_y(i) = ipp.P[i-1][1];
            c_z(i) = ipp.P[i-1][2];
        }
    }

    Eigen::VectorXd b_x = Eigen::VectorXd::Random(res_size);
    Eigen::VectorXd b_y = Eigen::VectorXd::Random(res_size);
    Eigen::VectorXd b_z = Eigen::VectorXd::Random(res_size);

    b_x = A.colPivHouseholderQr().solve(c_x);
    b_y = A.colPivHouseholderQr().solve(c_y);
    b_z = A.colPivHouseholderQr().solve(c_z);

    return {{b_x[0], b_y[0], b_z[0]},{b_x[1], b_y[1], b_z[1]},{b_x[2], b_y[2], b_z[2]},{b_x[3], b_y[3], b_z[3]},{b_x[4], b_y[4], b_z[4]}};

}




int main()
{

    if (!glfwInit())
    {
        return -1;
    }

    GLFWwindow *window = glfwCreateWindow(1500, 1000, "fit1 app", nullptr, nullptr);

    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    InputPoints ip;
    
    ip.center = {3,0,0};
    ip.P =
    {
        {5,4,4},
        {5,4,-4},
        {5,-4,-4},
        {5,-4, 4}
    };

    ip.translatePoints();

    const auto surface_params = calcUVs(ip);

    const auto eq_params = calcDer(ip, surface_params);

    std::cout<<eq_params.Su<<"\n"<<eq_params.Sv<<"\n"<<eq_params.Suu<<"\n"<<eq_params.Suv<< "\n"<< eq_params.Svv<<std::endl;


    //for(auto a : surface_params){
    //    std::cout<<a.u<<" "<<a.v<<std::endl;
    //}

    while (!glfwWindowShouldClose(window))
    {

        glClear(GL_COLOR_BUFFER_BIT);

        
        



        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
