# See https://doc.qt.io/qt-5/cmake-get-started.html#build-a-gui-executable
#set(QT_PATH /opt/qt512/)
#message("QT_PATH: ${QT_PATH}")
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${QT_PATH})

# Enable auto-generation of GUI files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5
      COMPONENTS
        Core
        Concurrent
        Gui
        Widgets
        Network
        PrintSupport
        OpenGL
        Xml
        Test # required by OpenCV HighGUI when compiled with QT support
        Charts # requires "sudo apt install libqt5charts5-dev"
        SerialPort # requires "sudo apt install libqt5serialport5-dev"
        3DCore # requires "sudo apt install qt3d* qt3d5-dev"
        3DExtras
        3DRender
        3DInput        
      REQUIRED)