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
        packageFun = import ./Cargo.nix;
      };
      
      rigidbody = (rustPkgs.workspace.rigidbody {});
      rigidbody_bindings = (rustPkgs.workspace.rigidbody-bindings {});

      bindings = pkgs.stdenv.mkDerivation {
        pname = "rigidbody-bindings";
        version = "0.0.1";
        src = ./rigidbody_bindings;

        buildInputs = [
          pkgs.gcc
        ];

        propagatedBuildInputs = [
          pkgs.pinocchio
          pkgs.eigen
          rigidbody_bindings
        ];

        configurePhase = "";
        buildPhase = ''
          mkdir -p $out/bin
          g++ -o main main.cpp -L${rigidbody_bindings}/bin -I${pkgs.eigen}/include/eigen3 -L${pkgs.pinocchio}/lib -lpinocchio_default -lpinocchio_parsers -lrigidbody_bindings
        '';

        installPhase = ''
          mkdir -p $out/bin
          cp main $out/bin/
        '';
      };
    in
      {
        packages.${system}.default = bindings;
        apps.${system} = {
          default = {type="app"; program="${bindings}/bin/main";};      # runs cpp/main.cpp, linked again rigidboy_binings.so
          rigidbody =  {type="app"; program="${rigidbody}/bin/rigidbody";}; # runs rigibody/src/main.rs 
        };
        devShells.${system} = {        
          default = pkgs.mkShell {
            packages = [
              rigidbody
              rigidbody_bindings
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
