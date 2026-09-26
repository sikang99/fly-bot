#
# Makefile for fly-bot
#
all:
	@echo "usage: make [run]"

run r:
	@echo "> make (run) [dev|web]"

run-dev rd:
	npm ci
	npm run dev


run-web rw:
	@echo "> make (run-web) [open|flat|rough]"

run-web-open rwo:
	open http://localhost:5173/a2.html
run-web-flat rwf:
	open http://localhost:5173/a2.html?terrain=flat
run-web-rough rwr:
	open http://localhost:5173/a2.html?terrain=rough


#---------------------------------------------------
VERSION=1.0.1
git -update gu:
	git add .
	git commit -a -m "v$(VERSION),$(USER)"
	git push
