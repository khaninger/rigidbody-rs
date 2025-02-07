{
  description = "Dependencies for benchmarking Pinocchio";

  inputs = {
    cargo2nix.url = "github:cargo2nix/cargo2nix"; 
    nixpkgs.url =  "github:nixos/nixpkgs/nixos-24.11";
  };

  outputs = { self, cargo2nix, nixpkgs }:
    let
      system = "x86_64-linux";
      overlays = [
        cargo2nix.overlays.default
        (final: prev: {
          rustc = prev.rustc.overrideAttrs (old: {
            src = pkgs.fetchFromGitHub {
              owner = "EnzymeAD";
              repo = "rust";
              rev = "master"; # Replace with specific commit/tag if needed
              sha256 = "sha256-MlK2ss22AGOzrnvJXgsqD1177QGurjlvRC5gluDfFzI="; # Replace after first build
              #depth = 1;
            };
          });
        })
      ];
      
      pkgs = import nixpkgs {
        inherit system overlays;
      };

      rustPkgs = pkgs.rustBuilder.makePackageSet {
        rustVersion = "1.75.0";
        packageFun = import ./Cargo.nix;
      };
      
      rigidbody = (rustPkgs.workspace.rigidbody {});
      rigidbody_bindings = (rustPkgs.workspace.rigidbody-bindings {});

      bindings = pkgs.stdenv.mkDerivation {
        pname = "rigidbody-bindings";
        version = "0.0.1";
        src = ./rigidbody_bindings;

        #buildInputs = [ pkgs.gcc ];

        buildInputs = [pkgs.cmake];
        propagatedBuildInputs = [
          pkgs.pinocchio
          pkgs.eigen
          rigidbody_bindings
        ];
      };
    in
      {
        packages.${system} = {
          default = bindings;
          enzyme = pkgs.rustc;
        };
        apps.${system} = {
          default = {type="app"; program="${bindings}/bin/main";};      # runs cpp/main.cpp, linked again rigidboy_binings.so
          rigidbody =  {type="app"; program="${rigidbody}/bin/rigidbody";}; # runs rigibody/src/main.rs 
        };
        devShells.${system} = {        
          default = pkgs.mkShell {
            buildInputs = [
              rigidbody_bindings
              pkgs.pinocchio
              pkgs.eigen
            ];
            shellHook = ''
                export CMAKE_PREFIX_PATH=${pkgs.pinocchio}/lib/cmake/pinocchio:${pkgs.eigen}/share/eigen3/cmake:$CMAKE_PREFIX_PATH
            ''; 
          };
          enzyme = pkgs.mkShell { buildInputs = [ pkgs.rustc ]; };
        };
      };
  nixConfig = {
    bash-prompt-prefix = "\\[\\e[38;5;12m\\](nix)\\[\\e[38;5;220m\\] ";
    extra-substituters = [
      "https://nix-community.cachix.org"
    ];
    extra-trusted-public-keys = [
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
    ];
  };
}
