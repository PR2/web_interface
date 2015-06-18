all: apache.cfg setup.bash

apache.cfg: apache.cfg.in
	bash apache.cfg.in > apache.cfg

setup.bash: setup.bash.in
	bash setup.bash.in > setup.bash
