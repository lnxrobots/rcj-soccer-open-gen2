workspace "LNXVisualizer"
    architecture "x86_64"
    startproject "LNXVisualizer"

    configurations
    {
        "Debug",
        "Release"
    }

    flags
    {
        "MultiProcessorCompile"
    }

    outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

    IncludeDir = {}
    IncludeDir["GLFW"] = "%{wks.location}/vendor/GLFW/include"
    IncludeDir["Glad"] = "%{wks.location}/vendor/Glad/include"
    IncludeDir["imgui"] = "%{wks.location}/vendor/imgui"
    IncludeDir["stb_image"] = "%{wks.location}/vendor/stb_image"
    IncludeDir["asio"] = "%{wks.location}/vendor/asio/include"
    IncludeDir["websocketpp"] = "%{wks.location}/vendor/websocketpp/include"

    group "Dependencies"
	    include "vendor/GLFW"
        include "vendor/Glad"
        include "vendor/imgui"
    group ""

    project "LNXVisualizer"
        kind "ConsoleApp"
        language "C++"
        cppdialect "C++17"

        targetdir ("%{wks.location}/bin/" .. outputdir .. "/%{prj.name}")
        objdir ("%{wks.location}/bin-int/" .. outputdir .. "/%{prj.name}")

        files
        {
            "src/**.cpp",
            "src/**.hpp"
        }

        includedirs
        {
            "src",
            "%{IncludeDir.GLFW}",
            "%{IncludeDir.Glad}",
            "%{IncludeDir.imgui}",
            "%{IncludeDir.stb_image}",
            "%{IncludeDir.websocketpp}"
        }

        -- Will be deprecated, use "externalincludedirs" when update to premake 6
        sysincludedirs
        {
            "%{IncludeDir.asio}"
        }

        defines
        {
            "GLFW_INCLUDE_NONE",
            "ASIO_STANDALONE"
        }

        links
        {
            "GLFW",
            "Glad",
            "imgui",
        }

        filter "system:windows"
            systemversion "latest"

            defines
            {
                "LNXVIS_PLATFORM_WIN",

                "_WEBSOCKETPP_CPP11_STRICT_"
            }

            links
            {
                "opengl32.lib"
            }

        filter "system:macosx"
            defines
            {
                "LNXVIS_PLATFORM_OSX"
            }

            links
            {
                "OpenGL.framework",
                "Cocoa.framework",
                "IOKit.framework"
            }

        filter "configurations:Debug"
            defines
            {
                "LNXVIS_DEBUG"
            }

		    runtime "Debug"
		    symbols "on"

	    filter "configurations:Release"
            defines
            {
                "LNXVIS_RELEASE"
            }

		    runtime "Release"
		    optimize "on"
