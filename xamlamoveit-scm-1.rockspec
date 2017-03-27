package = "xamlamoveit"
version = "scm-1"

source = {
   url = "",
}

description = {
   summary = "Moveit utilities for Rosvita control",
   detailed = [[
   ]],
   homepage = "https://github.com/Xamla/Rosvita.Control",
   license = ""
}

dependencies = {
   "torch >= 7.0",
   "torch-moveit"
}

build = {
   type = "command",
   build_command = [[
cmake -E make_directory build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$(PREFIX)" && $(MAKE)
]],
   install_command = "cd build && $(MAKE) install"
}
