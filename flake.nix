{
  description = "Graph Theory Testing for Robotics";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs";
  };

  outputs = { nixpkgs, ... }: let
    system = "x86_64-linux";
    pkgs = import nixpkgs { inherit system; };
    lib = pkgs.lib;
  in {
    devShells.${system}.default = pkgs.mkShell {
      name = "graphtest";
      packages = with pkgs; [
        jdk21
        jdt-language-server
      ];

      DISPLAY = ":0";
      LD_LIBRARY_PATH = "./build/jni/release:" + (lib.makeLibraryPath [ pkgs.libGL ]);
      HALSIM_EXTENSIONS = "./build/jni/release/libhalsim_gui.so";
    };
  };
}
