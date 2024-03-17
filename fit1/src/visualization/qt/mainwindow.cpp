#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "canvas.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QObject::connect(ui->yawplus, &QPushButton::pressed, this, &MainWindow::yawPlus);
    QObject::connect(ui->yawminus, &QPushButton::pressed, this, &MainWindow::yawMinus);
    QObject::connect(ui->pitchplus, &QPushButton::pressed, this, &MainWindow::pitchPlus);
    QObject::connect(ui->pitchminus, &QPushButton::pressed, this, &MainWindow::pitchMinus);
    QObject::connect(ui->rollplus, &QPushButton::pressed, this, &MainWindow::rollPlus);
    QObject::connect(ui->rollminus, &QPushButton::pressed, this, &MainWindow::rollMinus);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::yawPlus()
{
    ui->main_openGL_widget->changeYaw(0.2);
    ui->main_openGL_widget->update();
}
void MainWindow::yawMinus()
{
    ui->main_openGL_widget->changeYaw(-0.2);
    ui->main_openGL_widget->update();
}

void MainWindow::pitchPlus()
{
    ui->main_openGL_widget->changePitch(0.2);
    ui->main_openGL_widget->update();
}
void MainWindow::pitchMinus()
{
    ui->main_openGL_widget->changePitch(-0.2);
    ui->main_openGL_widget->update();
}

void MainWindow::rollPlus()
{
    ui->main_openGL_widget->changeRoll(0.2);
    ui->main_openGL_widget->update();
}
void MainWindow::rollMinus()
{
    ui->main_openGL_widget->changeRoll(-0.2);
    ui->main_openGL_widget->update();
}