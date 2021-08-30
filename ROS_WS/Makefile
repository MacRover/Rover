mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
current_dir := $(notdir $(patsubst %/,%,$(dir $(mkfile_path))))

help: ## Show all Makefile targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

init: ## setup
ifneq ($(wildcard $(current_dir)/build),)
		catkin_make
else
		catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python2 -DBOOST_ROOT=/opt/boost1.69 -DBoost_NO_SYSTEM_PATHS=TRUE
endif

clean: ## cleanup
	rm -rf build devel
