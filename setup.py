from setuptools import find_packages
from sys import platform as _platform
import sys
import glob
import os

from distutils.core import setup
from distutils.extension import Extension
from distutils.util import get_platform
from glob import glob

# Import pybind11
from pybind11.setup_helpers import Pybind11Extension, build_ext
from pybind11 import get_cmake_dir
import pybind11

# monkey-patch for parallel compilation
import multiprocessing
import multiprocessing.pool


def parallelCCompile(self,
                     sources,
                     output_dir=None,
                     macros=None,
                     include_dirs=None,
                     debug=0,
                     extra_preargs=None,
                     extra_postargs=None,
                     depends=None):
    # those lines are copied from distutils.ccompiler.CCompiler directly
    macros, objects, extra_postargs, pp_opts, build = self._setup_compile(
        output_dir, macros, include_dirs, sources, depends, extra_postargs)
    cc_args = self._get_cc_args(pp_opts, debug, extra_preargs)
    # parallel code
    N = 2 * multiprocessing.cpu_count()  # number of parallel compilations
    try:
        # On Unix-like platforms attempt to obtain the total memory in the
        # machine and limit the number of parallel jobs to the number of Gbs
        # of RAM (to avoid killing smaller platforms like the Pi)
        mem = os.sysconf('SC_PHYS_PAGES') * os.sysconf('SC_PAGE_SIZE')  # bytes
    except (AttributeError, ValueError):
        # Couldn't query RAM; don't limit parallelism (it's probably a well
        # equipped Windows / Mac OS X box)
        pass
    else:
        mem = max(1, int(round(mem / 1024 ** 3)))  # convert to Gb
        N = min(mem, N)

    def _single_compile(obj):
        try:
            src, ext = build[obj]
        except KeyError:
            return
        newcc_args = cc_args
        if _platform == "darwin":
            if src.endswith('.cpp'):
                newcc_args = cc_args + ["-mmacosx-version-min=10.7", "-stdlib=libc++"]
        self._compile(obj, src, ext, newcc_args, extra_postargs, pp_opts)

    # convert to list, imap is evaluated on-demand
    pool = multiprocessing.pool.ThreadPool(N)
    list(pool.imap(_single_compile, objects))
    return objects


import distutils.ccompiler

distutils.ccompiler.CCompiler.compile = parallelCCompile

# see http://stackoverflow.com/a/8719066/295157
import os

platform = get_platform()
print(platform)

CXX_FLAGS = ''
CXX_FLAGS += '-DGWEN_COMPILE_STATIC '
CXX_FLAGS += '-DBT_USE_DOUBLE_PRECISION '
CXX_FLAGS += '-DBT_ENABLE_ENET '
CXX_FLAGS += '-DBT_ENABLE_CLSOCKET '
CXX_FLAGS += '-DB3_DUMP_PYTHON_VERSION '
CXX_FLAGS += '-DEGL_ADD_PYTHON_INIT '
CXX_FLAGS += '-DB3_ENABLE_FILEIO_PLUGIN '
CXX_FLAGS += '-DB3_USE_ZIPFILE_FILEIO '
CXX_FLAGS += '-DBT_THREADSAFE=1 '
CXX_FLAGS += '-DSTATIC_LINK_SPD_PLUGIN '
CXX_FLAGS += '-DBT_ENABLE_VHACD '

EGL_CXX_FLAGS = ''

# libraries += [current_python]

libraries = []
include_dirs = []

try:
    import numpy

    NP_DIRS = [numpy.get_include()]
except:
    print("numpy is disabled. getCameraImage maybe slower.")
else:
    print("numpy is enabled.")
    CXX_FLAGS += '-DPYBULLET_USE_NUMPY '
    for d in NP_DIRS:
        print("numpy_include_dirs = %s" % d)
    include_dirs += NP_DIRS

# Collect all .cpp files from better_python directory
better_python_sources = []
better_python_dir = "better_python"
for root, dirs, files in os.walk(better_python_dir):
    for file in files:
        if file.endswith('.cpp') or file.endswith('.cc') or file.endswith('.cxx'):
            better_python_sources.append(os.path.join(root, file))

print(f"Found {len(better_python_sources)} pybind11 source files in {better_python_dir}")

# Original bullet physics sources
bullet_sources = ["examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp"] \
                 + ["examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.cpp"] \
                 + ["examples/SharedMemory/plugins/stablePDPlugin/KinTree.cpp"] \
                 + ["examples/SharedMemory/plugins/stablePDPlugin/MathUtil.cpp"] \
                 + ["examples/SharedMemory/plugins/stablePDPlugin/RBDModel.cpp"] \
                 + ["examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.cpp"] \
                 + ["examples/SharedMemory/plugins/stablePDPlugin/Shape.cpp"] \
                 + ["examples/SharedMemory/plugins/stablePDPlugin/SpAlg.cpp"] \
                 + ["src/btLinearMathAll.cpp"] \
                 + ["src/btBulletCollisionAll.cpp"] \
                 + ["src/btBulletDynamicsAll.cpp"] \
                 + ["examples/ExampleBrowser/InProcessExampleBrowser.cpp"] \
                 + ["examples/TinyRenderer/geometry.cpp"] \
                 + ["examples/TinyRenderer/model.cpp"] \
                 + ["examples/TinyRenderer/tgaimage.cpp"] \
                 + ["examples/TinyRenderer/our_gl.cpp"] \
                 + ["examples/TinyRenderer/TinyRenderer.cpp"] \
                 + ["examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp"] \
                 + ["examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp"] \
                 + ["examples/SharedMemory/plugins/fileIOPlugin/fileIOPlugin.cpp"] \
                 + ["examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp"] \
                 + ["examples/SharedMemory/IKTrajectoryHelper.cpp"] \
                 + ["examples/SharedMemory/InProcessMemory.cpp"] \
                 + ["examples/SharedMemory/PhysicsClient.cpp"] \
                 + ["examples/SharedMemory/PhysicsServer.cpp"] \
                 + ["examples/SharedMemory/GraphicsClientExample.cpp"] \
                 + ["examples/SharedMemory/GraphicsServerExample.cpp"] \
                 + ["examples/SharedMemory/RemoteGUIHelper.cpp"] \
                 + ["examples/SharedMemory/RemoteGUIHelperTCP.cpp"] \
                 + ["examples/SharedMemory/PhysicsServerExample.cpp"] \
                 + ["examples/SharedMemory/PhysicsServerExampleBullet2.cpp"] \
                 + ["examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp"] \
                 + ["examples/SharedMemory/PhysicsServerSharedMemory.cpp"] \
                 + ["examples/SharedMemory/PhysicsDirect.cpp"] \
                 + ["examples/SharedMemory/PhysicsDirectC_API.cpp"] \
                 + ["examples/SharedMemory/PhysicsServerCommandProcessor.cpp"] \
                 + ["examples/SharedMemory/PhysicsClientSharedMemory.cpp"] \
                 + ["examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp"] \
                 + ["examples/SharedMemory/PhysicsClientC_API.cpp"] \
                 + ["examples/SharedMemory/Win32SharedMemory.cpp"] \
                 + ["examples/SharedMemory/PosixSharedMemory.cpp"] \
                 + ["examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp"] \
                 + ["examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp"] \
                 + ["examples/SharedMemory/PhysicsClientUDP.cpp"] \
                 + ["examples/SharedMemory/PhysicsClientUDP_C_API.cpp"] \
                 + ["examples/SharedMemory/PhysicsClientTCP.cpp"] \
                 + ["examples/SharedMemory/PhysicsClientTCP_C_API.cpp"] \
                 + ["examples/SharedMemory/b3PluginManager.cpp"] \
                 + ["examples/Utils/b3ResourcePath.cpp"] \
                 + ["examples/Utils/RobotLoggingUtil.cpp"] \
                 + ["examples/Utils/ChromeTraceUtil.cpp"] \
                 + ["examples/Utils/b3Clock.cpp"] \
                 + ["examples/Utils/b3Quickprof.cpp"] \
                 + ["examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp"] \
                 + ["examples/ThirdPartyLibs/stb_image/stb_image.cpp"] \
                 + ["examples/ThirdPartyLibs/stb_image/stb_image_write.cpp"] \
                 + ["examples/ThirdPartyLibs/minizip/ioapi.c"] \
                 + ["examples/ThirdPartyLibs/minizip/unzip.c"] \
                 + ["examples/ThirdPartyLibs/minizip/zip.c"] \
                 + ["examples/ThirdPartyLibs/zlib/adler32.c"] \
                 + ["examples/ThirdPartyLibs/zlib/compress.c"] \
                 + ["examples/ThirdPartyLibs/zlib/crc32.c"] \
                 + ["examples/ThirdPartyLibs/zlib/deflate.c"] \
                 + ["examples/ThirdPartyLibs/zlib/gzclose.c"] \
                 + ["examples/ThirdPartyLibs/zlib/gzlib.c"] \
                 + ["examples/ThirdPartyLibs/zlib/gzread.c"] \
                 + ["examples/ThirdPartyLibs/zlib/gzwrite.c"] \
                 + ["examples/ThirdPartyLibs/zlib/infback.c"] \
                 + ["examples/ThirdPartyLibs/zlib/inffast.c"] \
                 + ["examples/ThirdPartyLibs/zlib/inflate.c"] \
                 + ["examples/ThirdPartyLibs/zlib/inftrees.c"] \
                 + ["examples/ThirdPartyLibs/zlib/trees.c"] \
                 + ["examples/ThirdPartyLibs/zlib/uncompr.c"] \
                 + ["examples/ThirdPartyLibs/zlib/zutil.c"] \
                 + ["examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp"] \
                 + ["examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp"] \
                 + ["examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp"] \
                 + ["examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp"] \
                 + ["examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp"] \
                 + ["examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp"] \
                 + ["examples/Importers/ImportURDFDemo/URDF2Bullet.cpp"] \
                 + ["examples/Importers/ImportURDFDemo/UrdfParser.cpp"] \
                 + ["examples/Importers/ImportURDFDemo/urdfStringSplit.cpp"] \
                 + ["examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp"] \
                 + ["examples/MultiThreading/b3PosixThreadSupport.cpp"] \
                 + ["examples/MultiThreading/b3Win32ThreadSupport.cpp"] \
                 + ["examples/MultiThreading/b3ThreadSupportInterface.cpp"] \
                 + ["examples/ThirdPartyLibs/enet/callbacks.c"] \
                 + ["examples/ThirdPartyLibs/enet/compress.c"] \
                 + ["examples/ThirdPartyLibs/enet/host.c"] \
                 + ["examples/ThirdPartyLibs/enet/list.c"] \
                 + ["examples/ThirdPartyLibs/enet/packet.c"] \
                 + ["examples/ThirdPartyLibs/enet/peer.c"] \
                 + ["examples/ThirdPartyLibs/enet/protocol.c"] \
                 + ["examples/ExampleBrowser/OpenGLGuiHelper.cpp"] \
                 + ["examples/ExampleBrowser/OpenGLExampleBrowser.cpp"] \
                 + ["examples/ExampleBrowser/CollisionShape2TriangleMesh.cpp"] \
                 + ["examples/ExampleBrowser/GL_ShapeDrawer.cpp"] \
                 + ["examples/OpenGLWindow/SimpleOpenGL2Renderer.cpp"] \
                 + ["examples/OpenGLWindow/GLInstancingRenderer.cpp"] \
                 + ["examples/OpenGLWindow/SimpleOpenGL3App.cpp"] \
                 + ["examples/OpenGLWindow/GLPrimitiveRenderer.cpp"] \
                 + ["examples/OpenGLWindow/TwFonts.cpp"] \
                 + ["examples/OpenGLWindow/GLRenderToTexture.cpp"] \
                 + ["examples/OpenGLWindow/LoadShader.cpp"] \
                 + ["examples/OpenGLWindow/OpenSans.cpp"] \
                 + ["examples/OpenGLWindow/SimpleCamera.cpp"] \
                 + ["examples/OpenGLWindow/fontstash.cpp"] \
                 + ["examples/OpenGLWindow/SimpleOpenGL2App.cpp"] \
                 + ["examples/OpenGLWindow/opengl_fontstashcallbacks.cpp"] \
                 + ["examples/ExampleBrowser/GwenGUISupport/GraphingTexture.cpp"] \
                 + ["examples/ExampleBrowser/GwenGUISupport/GwenProfileWindow.cpp"] \
                 + ["examples/ExampleBrowser/GwenGUISupport/gwenUserInterface.cpp"] \
                 + ["examples/ExampleBrowser/GwenGUISupport/GwenParameterInterface.cpp"] \
                 + ["examples/ExampleBrowser/GwenGUISupport/GwenTextureWindow.cpp"] \
                 + ["src/Bullet3Common/b3AlignedAllocator.cpp"] \
                 + ["src/Bullet3Common/b3Logging.cpp"] \
                 + ["src/Bullet3Common/b3Vector3.cpp"] \
                 + ["examples/ThirdPartyLibs/clsocket/src/ActiveSocket.cpp"] \
                 + ["examples/ThirdPartyLibs/clsocket/src/PassiveSocket.cpp"] \
                 + ["examples/ThirdPartyLibs/clsocket/src/SimpleSocket.cpp"] \
                 + ["Extras/Serialize/BulletFileLoader/bChunk.cpp"] \
                 + ["Extras/Serialize/BulletFileLoader/bDNA.cpp"] \
                 + ["Extras/Serialize/BulletFileLoader/bFile.cpp"] \
                 + ["Extras/Serialize/BulletFileLoader/btBulletFile.cpp"] \
                 + ["Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.cpp"] \
                 + ["Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp"] \
                 + ["Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp"] \
                 + ["Extras/InverseDynamics/CloneTreeCreator.cpp"] \
                 + ["Extras/InverseDynamics/IDRandomUtil.cpp"] \
                 + ["Extras/InverseDynamics/MultiBodyTreeDebugGraph.cpp"] \
                 + ["Extras/InverseDynamics/User2InternalIndex.cpp"] \
                 + ["Extras/InverseDynamics/CoilCreator.cpp"] \
                 + ["Extras/InverseDynamics/MultiBodyNameMap.cpp"] \
                 + ["Extras/InverseDynamics/RandomTreeCreator.cpp"] \
                 + ["Extras/InverseDynamics/btMultiBodyTreeCreator.cpp"] \
                 + ["Extras/InverseDynamics/DillCreator.cpp"] \
                 + ["Extras/InverseDynamics/MultiBodyTreeCreator.cpp"] \
                 + ["Extras/InverseDynamics/SimpleTreeCreator.cpp"] \
                 + ["Extras/InverseDynamics/invdyn_bullet_comparison.cpp"] \
                 + ["src/BulletSoftBody/btDefaultSoftBodySolver.cpp"] \
                 + ["src/BulletSoftBody/btSoftBodyHelpers.cpp"] \
                 + ["src/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp"] \
                 + ["src/BulletSoftBody/btSoftBody.cpp"] \
                 + ["src/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp"] \
                 + ["src/BulletSoftBody/btSoftRigidDynamicsWorld.cpp"] \
                 + ["src/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp"] \
                 + ["src/BulletSoftBody/btSoftMultiBodyDynamicsWorld.cpp"] \
                 + ["src/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp"] \
                 + ["src/BulletSoftBody/btDeformableBackwardEulerObjective.cpp"] \
                 + ["src/BulletSoftBody/btDeformableBodySolver.cpp"] \
                 + ["src/BulletSoftBody/btDeformableContactProjection.cpp"] \
                 + ["src/BulletSoftBody/btDeformableContactConstraint.cpp"] \
                 + ["src/BulletSoftBody/btDeformableMultiBodyConstraintSolver.cpp"] \
                 + ["src/BulletSoftBody/btDeformableMultiBodyDynamicsWorld.cpp"] \
                 + ["src/BulletSoftBody/poly34.cpp"] \
                 + ["src/BulletInverseDynamics/IDMath.cpp"] \
                 + ["src/BulletInverseDynamics/MultiBodyTree.cpp"] \
                 + ["src/BulletInverseDynamics/details/MultiBodyTreeImpl.cpp"] \
                 + ["src/BulletInverseDynamics/details/MultiBodyTreeInitCache.cpp"] \
                 + ["examples/ThirdPartyLibs/BussIK/Jacobian.cpp"] \
                 + ["examples/ThirdPartyLibs/BussIK/LinearR2.cpp"] \
                 + ["examples/ThirdPartyLibs/BussIK/LinearR3.cpp"] \
                 + ["examples/ThirdPartyLibs/BussIK/LinearR4.cpp"] \
                 + ["examples/ThirdPartyLibs/BussIK/MatrixRmn.cpp"] \
                 + ["examples/ThirdPartyLibs/BussIK/Misc.cpp"] \
                 + ["examples/ThirdPartyLibs/BussIK/Node.cpp"] \
                 + ["examples/ThirdPartyLibs/BussIK/Tree.cpp"] \
                 + ["examples/ThirdPartyLibs/BussIK/VectorRn.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Anim.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/DragAndDrop.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Hook.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/ToolTip.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/events.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/BaseRender.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Gwen.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Skin.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Utility.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/inputhandler.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Base.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Button.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Canvas.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/CheckBox.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ColorControls.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ColorPicker.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ComboBox.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/CrossSplitter.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/DockBase.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/DockedTabControl.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Dragger.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/GroupBox.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/HSVColorPicker.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/HorizontalScrollBar.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ImagePanel.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/HorizontalSlider.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Label.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/LabelClickable.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ListBox.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/MenuItem.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Menu.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/MenuStrip.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/NumericUpDown.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/PanelListPanel.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ProgressBar.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Properties.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/RadioButton.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/RadioButtonController.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ResizableControl.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Resizer.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/RichLabel.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ScrollBar.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ScrollBarBar.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ScrollBarButton.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/ScrollControl.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Slider.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/SplitterBar.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/TabButton.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/TabControl.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/TabStrip.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Text.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/TextBox.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/TextBoxNumeric.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/TreeControl.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/TreeNode.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/VerticalScrollBar.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/VerticalSlider.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/WindowControl.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Dialog/FileOpen.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Dialog/FileSave.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Controls/Dialog/Query.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Platforms/Null.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Platforms/Windows.cpp"] \
                 + ["examples/ThirdPartyLibs/Gwen/Renderers/OpenGL_DebugFont.cpp"] \
                 + ["Extras/VHACD/test/src/main_vhacd.cpp"] \
                 + ["Extras/VHACD/src/VHACD.cpp"] \
                 + ["Extras/VHACD/src/vhacdICHull.cpp"] \
                 + ["Extras/VHACD/src/vhacdManifoldMesh.cpp"] \
                 + ["Extras/VHACD/src/vhacdMesh.cpp"] \
                 + ["Extras/VHACD/src/vhacdVolume.cpp"]

# Combine pybind11 sources with bullet sources
sources = better_python_sources + bullet_sources

egl_renderer_sources = \
    ["examples/SharedMemory/plugins/eglPlugin/eglRendererVisualShapeConverter.cpp"] \
    + ["examples/SharedMemory/plugins/eglPlugin/eglRendererPlugin.cpp"] \
    + ["examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp"] \
    + ["examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp"] \
    + ["examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp"] \
    + ["examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp"] \
    + ["examples/TinyRenderer/geometry.cpp"] \
    + ["examples/TinyRenderer/model.cpp"] \
    + ["examples/TinyRenderer/tgaimage.cpp"] \
    + ["examples/TinyRenderer/our_gl.cpp"] \
    + ["examples/TinyRenderer/TinyRenderer.cpp"] \
    + ["examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp"] \
    + ["examples/ThirdPartyLibs/stb_image/stb_image.cpp"] \
    + ["examples/ThirdPartyLibs/stb_image/stb_image_write.cpp"] \
    + ["examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp"] \
    + ["examples/OpenGLWindow/SimpleCamera.cpp"] \
    + ["examples/Utils/b3Clock.cpp"] \
    + ["examples/Utils/b3ResourcePath.cpp"] \
    + ["src/BulletCollision/CollisionShapes/btShapeHull.cpp"] \
    + ["src/BulletCollision/CollisionShapes/btConvexHullShape.cpp"] \
    + ["src/BulletCollision/CollisionShapes/btBoxShape.cpp"] \
    + ["src/BulletCollision/CollisionShapes/btSphereShape.cpp"] \
    + ["src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp"] \
    + ["src/BulletCollision/CollisionShapes/btConvexShape.cpp"] \
    + ["src/BulletCollision/CollisionShapes/btCollisionShape.cpp"] \
    + ["src/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp"] \
    + ["src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp"] \
    + ["src/Bullet3Common/b3Logging.cpp"] \
    + ["src/LinearMath/btAlignedAllocator.cpp"] \
    + ["src/LinearMath/btConvexHull.cpp"] \
    + ["src/LinearMath/btConvexHullComputer.cpp"] \
    + ["src/LinearMath/btGeometryUtil.cpp"] \
    + ["src/LinearMath/btQuickprof.cpp"] \
    + ["src/LinearMath/btThreads.cpp"] \
    + ["src/Bullet3Common/b3AlignedAllocator.cpp"] \
    + ["examples/ThirdPartyLibs/glad/gl.c"] \
    + ["examples/OpenGLWindow/GLInstancingRenderer.cpp"] \
    + ["examples/OpenGLWindow/GLRenderToTexture.cpp"] \
    + ["examples/OpenGLWindow/LoadShader.cpp"]

if 'BT_USE_EGL' in CXX_FLAGS:
    sources += ['examples/ThirdPartyLibs/glad/egl.c']
    sources += ['examples/OpenGLWindow/EGLOpenGLWindow.cpp']

if _platform == "linux" or _platform == "linux2":
    libraries = ['dl', 'pthread']
    CXX_FLAGS += '-D_LINUX '
    CXX_FLAGS += '-DGLEW_STATIC '
    CXX_FLAGS += '-DGLEW_INIT_OPENGL11_FUNCTIONS=1 '
    CXX_FLAGS += '-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1 '
    CXX_FLAGS += '-DDYNAMIC_LOAD_X11_FUNCTIONS '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-fno-inline-functions-called-once '
    CXX_FLAGS += '-fvisibility=hidden '
    CXX_FLAGS += '-fvisibility-inlines-hidden '
    EGL_CXX_FLAGS += '-DBT_USE_EGL '
    EGL_CXX_FLAGS += '-fPIC '  # for plugins

    sources = sources + ["examples/ThirdPartyLibs/enet/unix.c"] \
              + ["examples/OpenGLWindow/X11OpenGLWindow.cpp"] \
              + ["examples/ThirdPartyLibs/glad/gl.c"] \
              + ["examples/ThirdPartyLibs/glad/glx.c"]
    include_dirs += ["examples/ThirdPartyLibs/optionalX11"]

    if 'BT_USE_EGL' in EGL_CXX_FLAGS:
        egl_renderer_sources = egl_renderer_sources \
                               + ["examples/OpenGLWindow/EGLOpenGLWindow.cpp"] \
                               + ['examples/ThirdPartyLibs/glad/egl.c']
    else:
        egl_renderer_sources = egl_renderer_sources \
                               + ["examples/OpenGLWindow/X11OpenGLWindow.cpp"] \
                               + ["examples/ThirdPartyLibs/glad/glx.c"]

elif _platform == "win32":
    print("win32!")
    libraries = ['Ws2_32', 'Winmm', 'User32', 'Opengl32', 'kernel32', 'glu32', 'Gdi32', 'Comdlg32']
    CXX_FLAGS += '-DWIN32 '
    CXX_FLAGS += '-DGLEW_STATIC '
    sources = sources + ["examples/ThirdPartyLibs/enet/win32.c"] \
              + ["examples/OpenGLWindow/Win32Window.cpp"] \
              + ["examples/OpenGLWindow/Win32OpenGLWindow.cpp"] \
              + ["examples/ThirdPartyLibs/glad/gl.c"]
elif _platform == "darwin":
    print("darwin!")
    os.environ['LDFLAGS'] = '-framework Cocoa -mmacosx-version-min=10.7 -stdlib=libc++ -framework OpenGL'
    CXX_FLAGS += '-DB3_NO_PYTHON_FRAMEWORK '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-D_DARWIN '
    #    CXX_FLAGS += '-framework Cocoa '
    sources = sources + ["examples/ThirdPartyLibs/enet/unix.c"] \
              + ["examples/OpenGLWindow/MacOpenGLWindow.cpp"] \
              + ["examples/ThirdPartyLibs/glad/gl.c"] \
              + ["examples/OpenGLWindow/MacOpenGLWindowObjC.m"]
else:
    print("bsd!")
    libraries = ['GL', 'GLEW', 'pthread']
    os.environ['LDFLAGS'] = '-L/usr/X11R6/lib'
    CXX_FLAGS += '-D_BSD '
    CXX_FLAGS += '-I/usr/X11R6/include '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-fno-inline-functions-called-once'
    sources = ["examples/ThirdPartyLibs/enet/unix.c"] \
              + ["examples/OpenGLWindow/X11OpenGLWindow.cpp"] \
              + ["examples/ThirdPartyLibs/glad/gl.c"] \
              + sources

setup_py_dir = os.path.dirname(os.path.realpath(__file__))

need_files = []
datadir = "examples/pybullet/gym/pybullet_data"

hh = setup_py_dir + "/" + datadir

for root, dirs, files in os.walk(hh):
    for fn in files:
        ext = os.path.splitext(fn)[1][1:]
        if ext and ext in 'yaml index meta data-00000-of-00001 png gif jpg urdf sdf obj txt mtl dae off stl STL xml gin npy '.split(
        ):
            fn = root + "/" + fn
            need_files.append(fn[1 + len(hh):])

print("found resource files: %i" % len(need_files))
for n in need_files:
    print("-- %s" % n)
print("packages")
print(find_packages('examples/pybullet/gym'))
print("-----")

extensions = []

# Create pybind11 extension instead of regular Extension
pybullet_ext = Pybind11Extension(
    "betterpybullet",
    sources=sources,
    libraries=libraries,
    cxx_std=11,  # C++11 or higher required for pybind11
    include_dirs=include_dirs + [
        "src",
        "examples/ThirdPartyLibs",
        "examples/ThirdPartyLibs/glad",
        "examples/ThirdPartyLibs/enet/include",
        "examples/ThirdPartyLibs/clsocket/src",
        "Extras/VHACD/inc",
        "Extras/VHACD/public",
        better_python_dir,  # Add your pybind11 directory to includes
        # pybind11 includes are automatically added by Pybind11Extension
    ],
    define_macros=[
        # Convert CXX_FLAGS to define_macros format
        ('GWEN_COMPILE_STATIC', None),
        ('BT_USE_DOUBLE_PRECISION', None),
        ('BT_ENABLE_ENET', None),
        ('BT_ENABLE_CLSOCKET', None),
        ('B3_DUMP_PYTHON_VERSION', None),
        ('EGL_ADD_PYTHON_INIT', None),
        ('B3_ENABLE_FILEIO_PLUGIN', None),
        ('B3_USE_ZIPFILE_FILEIO', None),
        ('BT_THREADSAFE', '1'),
        ('STATIC_LINK_SPD_PLUGIN', None),
        ('BT_ENABLE_VHACD', None),
    ]
)

# Add platform-specific defines
if _platform == "linux" or _platform == "linux2":
    pybullet_ext.define_macros.extend([
        ('_LINUX', None),
        ('GLEW_STATIC', None),
        ('GLEW_INIT_OPENGL11_FUNCTIONS', '1'),
        ('GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS', '1'),
        ('DYNAMIC_LOAD_X11_FUNCTIONS', None),
        ('HAS_SOCKLEN_T', None),
    ])
    # Add Linux-specific compiler flags
    pybullet_ext.extra_compile_args = [
        '-fno-inline-functions-called-once',
        '-fvisibility=hidden',
        '-fvisibility-inlines-hidden'
    ]
elif _platform == "win32":
    pybullet_ext.define_macros.extend([
        ('WIN32', None),
        ('GLEW_STATIC', None),
    ])
elif _platform == "darwin":
    pybullet_ext.define_macros.extend([
        ('B3_NO_PYTHON_FRAMEWORK', None),
        ('HAS_SOCKLEN_T', None),
        ('_DARWIN', None),
    ])
    pybullet_ext.extra_compile_args = [
        '-mmacosx-version-min=10.7',
        '-stdlib=libc++'
    ]
else:  # BSD
    pybullet_ext.define_macros.extend([
        ('_BSD', None),
        ('HAS_SOCKLEN_T', None),
    ])
    pybullet_ext.extra_compile_args = [
        '-I/usr/X11R6/include',
        '-fno-inline-functions-called-once'
    ]

# Add numpy defines if available
if 'PYBULLET_USE_NUMPY' in CXX_FLAGS:
    pybullet_ext.define_macros.append(('PYBULLET_USE_NUMPY', None))

extensions.append(pybullet_ext)

if 'BT_USE_EGL' in EGL_CXX_FLAGS:
    # Create EGL renderer extension with pybind11 if needed
    eglRender = Pybind11Extension(
        "eglRenderer",
        sources=egl_renderer_sources,
        libraries=libraries,
        cxx_std=11,
        include_dirs=include_dirs + [
            "src",
            "examples",
            "examples/ThirdPartyLibs",
            "examples/ThirdPartyLibs/glad",
            "examples/ThirdPartyLibs/enet/include",
            "examples/ThirdPartyLibs/clsocket/src"
        ],
        define_macros=[
            ('BT_USE_EGL', None),
        ]
    )

    # Add the same platform-specific settings to EGL renderer
    if _platform == "linux" or _platform == "linux2":
        eglRender.define_macros.extend([
            ('_LINUX', None),
            ('GLEW_STATIC', None),
            ('GLEW_INIT_OPENGL11_FUNCTIONS', '1'),
            ('GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS', '1'),
            ('DYNAMIC_LOAD_X11_FUNCTIONS', None),
            ('HAS_SOCKLEN_T', None),
        ])
        eglRender.extra_compile_args = [
            '-fPIC',
            '-fno-inline-functions-called-once',
            '-fvisibility=hidden',
            '-fvisibility-inlines-hidden'
        ]

    extensions.append(eglRender)

setup(
    name='betterpybullet',
    version='0.2.0',
    description=
    '',
    long_description=
    '',
    url='https://github.com/bulletphysics/bullet3',
    author='',
    author_email='',
    license='zlib',
    platforms='any',
    keywords=[
        'physics simulation', 'robotics',
        'collision detection', 'opengl'
    ],
    ext_modules=extensions,
    cmdclass={'build_ext': build_ext},  # Use pybind11's build_ext
    classifiers=[
        'License :: OSI Approved :: zlib/libpng License',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 3.8',
        'Topic :: Scientific/Engineering :: Artificial Intelligence :: Simulation',
        'Framework :: Robot Framework'
    ],
    python_requires='>=3.8',  # pybind11 requires Python 3.6+
    zip_safe=False,
)