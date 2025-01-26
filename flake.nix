{
  description = "Dependencies for benchmarking Pinocchio";

  inputs = {
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixpkgs.url =  "github:nixos/nixpkgs/nixos-24.11";
  };

  outputs = { self, fenix, nixpkgs }:
    let
      system = "x86_64-linux";
      overlays = [ fenix.overlays.default ];
      
      pkgs = import nixpkgs {
        inherit system overlays;
      };

      toolchain = fenix.packages.${system}.latest.toolchain;
      rustPlatform = pkgs.makeRustPlatform {
        cargo = toolchain;
        rustc = toolchain;
      };
      
      rigidbody = rustPlatform.buildRustPackage {
        pname = "rigidbody";
        version = "0.0.1";
        src = ./.;

        crateType = "cdylib";
        cargoLock.lockFile = ./Cargo.lock;
      };
      
      benchmark = pkgs.stdenv.mkDerivation {
        pname = "rigidbody-benchmark";
        version = "0.0.1";
        src = ./cpp;

        buildInputs = [
          pkgs.gcc
          pkgs.pinocchio
          pkgs.eigen
          rigidbody
        ];

        configurePhase = "";
        buildPhase = ''
          mkdir -p $out/bin
          echo ${rigidbody}
          ls ${rigidbody}/bin
          g++ -o main main.cpp -L${rigidbody}/bin -lrigidbody
        '';

        installPhase = ''
          mkdir -p $out/bin
          cp build/main $out/bin/
        '';
      };
    in
      {
        packages.${system}.default = benchmark;
        apps.${system} = {
          default = {type="app"; program="${benchmark}/bin/main";};
          #kinematics = {type = "app"; program="${benchmark}/bin/kinematics";};
          #rnea = {type = "app"; program="${benchmark}/bin/rnea";};
        };
        devShells = {        
          default = pkgs.mkShell {
            packages = [
              #benchmark
              rigidbody
            ];
          };
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
