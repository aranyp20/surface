#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "objectloader.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    framework::ObjectLoader object_loader;
		      
public slots:

    void yawPlus();
    void yawMinus();
    void pitchPlus();
    void pitchMinus();
    void rollPlus();
    void rollMinus();
  void setHighlightEdges(int status);
  void changeLoadedModel(int index);

};
#endif // MAINWINDOW_H
