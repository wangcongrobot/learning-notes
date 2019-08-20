

## Mujoco Model

//---------------------------------- mjModel --------------------------------------------

    mjtNum*   site_pos;             // local position offset rel. to body       (nsite x 3)
    mjtNum*   site_quat;            // local orientation offset rel. to body    (nsite x 4)



## site

- Site: Sites are essentially light geoms. They represent locations of interest within the body frame. Sites do not participate in collision detection or automated computation of inertial properties, however they can be used to specify the spatial properties of other objects: sensors, tendon routing, slider-crank endpoints.

- Tendon. Tendons are scalar length elements that can be used for actuation, imposing limits and equality constraints, or creating spring-dampers and friction loss. There are two types of tendons: fixed and spatial. Fixed tendons are linear combinations of (scalar) joint positions. They are useful for modeling mechanical coupling. Spatial tendons are defined as the shortest path that passes through a sequence of specified sites (or via-points) or wraps around specified geoms. Only spheres and cylinders are supported as wrapping geoms, and cylinders are treated as having infinite length for wrapping purposes. To avoid abrupt jumps of the tendon from one side of the wrapping geom to the other, the user can also specify the preferred side. If there are multiple wrapping geoms in the tendon path they must be separated by sites, so as to avoid the need for an iterative solver. Spatial tendons can also be split into multiple branches using pulleys.



## Bodies, geoms, sites

The results of forward kinematics are availabe in mjData as 
- xpos, xquat and xmat for bodies, 
- geom_xpos and geom_xmat for geoms, 
- site_xpos and site_xmat for sites.


bodies，geoms和sites是MuJoCo元素，大致对应于物理世界中的刚体。那他们为什么要分开呢？由于这里解释的语义和计算原因。 

首先是相似之处。bodies，几何体和场地都附有空间框架（尽管主体也有第二个框架，它以体心为中心并与惯性主轴对齐）。这些帧的位置和方向是在mjData.qpos通过正向运动学的每个时间步计算的。正向运动学的结果在mjData中可用作xpos，xquat和xmat用于body，geom_xpos和geom_xmat用于geoms，site_xpos和site_xmat用于站点。HAPTIX用户可以分别通过调用mj_get_body，mj_get_geom和mj_get_site来获取这些数量。 

现在的差异。bodies用于构建运动树，并且是其他元素的容器，包括geoms和sites。实体具有空间框架，惯性属性，但没有与外观或碰撞几何相关的属性。这是因为这些属性不会影响物理（当然除了联系人，但这些是单独处理的）。如果您已经在机器人教科书中看到了运动树图，那么这些物体通常被绘制成无定形形状 - 以表明它们的实际形状与物理学无关。 

Geoms（几何图元的简称）用于指定外观和碰撞几何。每个geom属于一个身体，并且严格地附着在那个身体上。多个geoms可以连接到同一个身体。考虑到MuJoCo的碰撞探测器假设所有的几何形状都是凸的（如果网格不是凸面，它在内部用它们的凸包替换网格），这一点特别有用。因此，如果要对非凸形状进行建模，则必须将其分解为凸形几何体的并集，并将它们全部附加到同一个体上。Geoms在XML模型中也可以具有质量和惯性（或者更确切地说，用于计算质量和ineria的材料密度），但这仅用于计算模型编译器中的体量和惯性。在模拟的实际mjModel中，geoms没有惯性属性。 

sites是轻的geoms。它们具有相同的外观特性但不能参与碰撞，不能用于推断体重。另一方面，站点可以执行geoms无法做到的事情：他们可以指定触摸传感器的体积，IMU传感器的附件，空间肌腱的路由，滑块 - 曲柄执行器的端点。这些都是空间量，但它们不对应应该具有质量或碰撞其他实体的实体 - 这就是创建网站元素的原因。站点还可用于指定用户感兴趣的点（或更确切的帧）。 

下面的例子说明了多个站点和geom可以连接到同一个主体的点：在这种情况下，两个站点和两个geom到一个主体。


## mocap

动作捕捉
Mocap体是世界上的静态子体（即没有关节），它们的mocap属性设置为“true”。它们可用于将来自运动捕捉设备的数据流输入到MuJoCo模拟中。假设您正在持有VR控制器或装有运动捕捉标记的对象（例如Vicon），并希望模拟对象以相同的方式移动，但也要与其他模拟对象进行交互。这里有一个两难境地：虚拟物体无法推动你的物理手，因此你的手（以及你控制的物体）可能会违反模拟物理。但与此同时，我们希望最终的模拟是合理的。我们如何做到这一点？ 

第一步是在MJCF模型中定义mocap主体，并在实现代码处在运行时读取数据流，并将mjModel.mocap_pos和mjModel.mocap_quat设置为从动作捕获系统接收的位置和方向。VR代码示例说明了如何在HTC Vive的情况下编写此类代码，而HATPIX在可用时自动从Optitrack系统流式传输数据。simulate.cpp代码示例使用鼠标作为动作捕捉设备，允许用户移动mocap体。 

关于mocap体的关键是模拟器将它们视为固定的。我们通过直接更新它们的位置和方向使它们从一个模拟时间步骤移动到下一个模拟时间步骤，但就物理模型而言，它们的位置和方向是恒定的。因此，如果我们与常规动态体接触会发生什么，如在MuJoCo 2.0分布中提供的复合对象示例中（回想一下，在那些示例中，我们有一个胶囊探针，它是我们用鼠标移动的mocap体）。两个规则体之间的接触将经历穿透以及相对速度，而与mocap体的接触缺少相对速度分量，因为模拟器不知道mocap体本身正在移动。因此产生的接触力较小，并且接触推动动态物体需要更长的时间。此外，在更复杂的模拟中，我们正在做与物理不一致的事情会导致不稳定。 

然而，在HAPTIX和VR代码示例中都使用了更好的替代方案。除了mocap体之外，我们还包括第二个常规体，并使用焊接等式约束将其连接到mocap体。在下面的图中，粉红色的盒子是mocap体，它连接到手的底部。在没有其他约束的情况下，手几乎完美地跟踪mocap体（并且比弹簧阻尼器更好），因为约束被隐式处理并且可以产生大的力而不会使模拟不稳定。但是，如果手被迫与桌子接触（例如右图），则不能同时尊重接触约束并跟踪mocap体。这是因为mocap体可以自由地穿过桌子。那么哪个约束赢了？这取决于对接触约束的实际焊接约束的柔软性。相应的需要调整solref和solimp参数，以实现所需的权衡。有关示例，请参阅HAPTIX发行版中的模块化假肢（MPL）手模型以及MuJoCo论坛; 下面的图是使用该模型生成的。 

## OpenGL

MuJoCo的原生OpenGL渲染器的使用将在渲染中解释。这里我们只讨论与外部库和解决依赖关系相关的问题。对于不需要渲染的项目，可以使用MuJoCo库的“nogl”版本。 

对于渲染MuJoCo使用OpenGL 1.5和ARB_framebuffer_object扩展（由所有现代驱动程序提供。）它还使用GLEW 2.0.0加载OpenGL符号。在Windows和macOS上，GLEW和OpenGL库（分别是OpenGL32.lib或OpenGL.framework）都与MuJoCo链接。在这些平台上，用户无需采取其他步骤来解决依赖关系。 

在Linux上，情况更复杂，因为有多个OpenGL实现和加载符号的方法。组织渲染器和链接过程以允许下面显示的多个用例。libmujoco200.so调​​用GLEW和OpenGL函数而不链接相应的库。用户必须通过链接任何适合他们需要的GLEW和OpenGL风格来解决依赖关系。有关说明，请参阅如何以三种不同的方式编译和链接Linux上的record.cpp（示例目录中的makefile）。

没有OpenGL：链接libmujoco200nogl .so
此版本的MuJoCo库是在没有渲染器的情况下编译的，因此它不会对OpenGL或GLEW进行任何调用。它可以在未安装图形驱动程序的计算服务器上使用。主头文件mujoco.h仍然声明了渲染函数（mjr_XXX），但是库没有实现它们，因此从用户代码调用这些函数将导致链接时未解析的符号。在代码示例中，我们在纯文本应用程序中使用此版本的库。
X11 OpenGL：链接libmujoco200 .so，libglew .so，libGL .so
这是在Linux桌面上使用OpenGL的最常用方法，X11用于上下文创建和符号加载。
OSMESA OpenGL：链接libmujoco200 .so，libglewosmesa .so，libOSMesa .so
这可用于使用OSMesa库进行软件渲染。请注意，我们在此处链接了不同版本的GLEW，使用OSMesa支持而不是标准GLX构建。理想情况下，用户将找到避免软件渲染和使用硬件加速的方法。但是当其他所有方法都失败时，OSMesa是一个方便的后备选项。
EGL OpenGL：链接libmujoco200 .so，libEGL .so，libglewegl .so，libOpenGL .so
这是无头渲染的关键功能。它可以在没有X11的服务器上实现硬件加速的OpenGL。用户代码必须使用EGL进行上下文创建，如record.cpp中所示。libglewegl.so使用EGL加载OpenGL符号而不是使用GLX。libOpenGL.so是独立于供应商的库，它只公开OpenGL功能而不引入对X11的依赖（而不是依赖于libGLX.so的libGL.so）。当使用“--install-libglvnd”选项安装时，NVidia的驱动程序提供libOpenGL.so。libEGL.so可以从Mesa获得，并且是独立于驱动程序的（sudo apt-get install libegl1-mesa-dev）。
最后，不是使用我们提供的已编译的libglew，用户可以自己编译，或者在项目中静态链接glew.c（它是一个单独的C文件）。静态链接需要定义GLEW_STATIC符号。此外，在为不同的用例构建GLEW 2.0.0时，必须定义以下符号：

  libglew.so:GLEW_NO_GLU
  libglewegl.so:GLEW_NO_GLU GLEW_EGL
  libgleweomesa.so:GLEW_NO_GLU GLEW_OSMESA
要使无头渲染工作，必须消除对GLU的依赖，否则将通过libGLU.so引入对X11的依赖。请注意，GLEW也可以为Mir和Wayland构建。我们还没有测试过这些，但理论上它们应该可行。

