{
  description = "Dependencies for benchmarking Pinocchio";

  inputs = {
    crane.url = "github:ipetkov/crane";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.rust-analyzer-src.follows = "";
    };
    nixpkgs.url =  "github:nixos/nixpkgs/nixos-24.11";
  };

  outputs = { self, crane, nixpkgs, fenix }:
    let
      system = "x86_64-linux";
      
      rustc-overlays = [
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
      overlays = [ fenix.overlays.default ];
      
      pkgs = import nixpkgs {
        inherit system overlays;
      };

      craneLib = (crane.mkLib pkgs).overrideToolchain(
        p: fenix.packages.${system}.complete.toolchain.overrideAttrs (oldAttrs: {
            src = pkgs.fetchFromGitHub {
              owner = "EnzymeAD";
              repo = "rust";
              rev = "master"; # Replace with specific commit/tag if needed
              sha256 = "sha256-MlK2ss22AGOzrnvJXgsqD1177QGurjlvRC5gluDfFzI="; # Replace after first build
            };
        })
      );
      
      src = craneLib.cleanCargoSource ./.;
      common = { inherit src;
                 strictDeps = true;
                 buildInputs = [ ]; };
      cargoArtifacts = craneLib.buildDepsOnly common;
      filesetForCrate = crate: pkgs.lib.fileset.toSource {
        root = ./.;
        fileset = pkgs.lib.fileset.unions [
          ./Cargo.toml
          ./Cargo.lock
          (craneLib.fileset.commonCargoSources ./rigidbody)
          (craneLib.fileset.commonCargoSources crate)
        ];
      };
      
      rigidbody = craneLib.buildPackage ( {
        pname = "rigidbody";
        inherit (craneLib.crateNameFromCargoToml { inherit src; }) version;
        inherit cargoArtifacts;
        doCheck = false;
        src = filesetForCrate ./rigidbody;
      });
      
      rigidbody_bindings = craneLib.buildPackage ({
        pname = "rigidbody-bindings";
        inherit (craneLib.crateNameFromCargoToml { inherit src; }) version;
        inherit cargoArtifacts;
        doCheck = false;
        src = filesetForCrate ./rigidbody_bindings;
      });

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
