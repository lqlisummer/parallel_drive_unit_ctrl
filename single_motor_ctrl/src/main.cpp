#include "app/cli_app.h"

int main(int argc, char** argv) {
    smc::CliApp app;
    return app.run(argc, argv);
}
