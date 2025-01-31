{
  description = "Dependencies for benchmarking Pinocchio";

  inputs = {
    cargo2nix.url = "github:cargo2nix/cargo2nix"; 
    nixpkgs.url =  "github:nixos/nixpkgs/nixos-24.11";
  };

  outputs = { self, cargo2nix, nixpkgs }:
    let
      system = "x86_64-linux";
      overlays = [ cargo2nix.overlays.default ];
      
      pkgs = import nixpkgs {
        inherit system overlays;
      };

      rustPkgs = pkgs.rustBuilder.makePackageSet {
        rustVersion = "1.75.0";
        packageFun = import ./rust_bindings/Cargo.nix;
      };
      
      #rigidbody = (rustPkgs.workspace.rigidbody {});
      rigidbody_bindings = (rustPkgs.workspace.rigidbody_bindings {});

      bindings = pkgs.stdenv.mkDerivation {
        pname = "rigidbody-bindings";
        version = "0.0.1";
        src = ./cpp;

        buildInputs = [
          pkgs.gcc
          pkgs.pinocchio
          pkgs.eigen
          rigidbody_bindings
        ];

        configurePhase = "";
        buildPhase = ''
          mkdir -p $out/bin
          g++ -o main main.cpp -L${rigidbody_bindings}/bin -lrigidbody_bindings
        '';

        installPhase = ''
          mkdir -p $out/bin
          cp build/main $out/bin/
        '';
      };

      benchmark = pkgs.stdenv.mkDerivation {
        pname = "rigidbody-benchmark";
        version = "0.0.1";
        src = ./cpp;

        buildInputs = [
          pkgs.cmake
          pkgs.gcc
          pkgs.pinocchio
          pkgs.eigen
        ];
      };        
    in
      {
        packages.${system}.default = benchmark;
        apps.${system} = {
          #default = {type="app"; program="${rigidbody}/bin/rigidbody";};
          bindings = {type="app"; program="${bindings}/bin/main";};
          kinematics = {type = "app"; program="${benchmark}/bin/kinematics";};
          rnea = {type = "app"; program="${benchmark}/bin/rnea";};
        };
        devShells = {        
          default = pkgs.mkShell {
            packages = [
              benchmark
              #rigidbody
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
