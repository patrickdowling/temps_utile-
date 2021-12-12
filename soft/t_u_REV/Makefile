ifdef VERBOSE
AT=
PIO_OPTS+=
else
AT=@
PIO_OPTS+=-s
endif

.PHONY: all
all: upload

.PHONY: run
run:
	$(AT)pio run $(PIO_OPTS)

.PHONY: upload
upload:
	$(AT)pio run $(PIO_OPTS) --target upload -e tu_dev

.PHONY: clean
clean:
	$(AT)pio run $(PIO_OPTS) --target clean -e tu_dev

.PHONY: prod
prod:
	$(AT)pio run $(PIO_OPTS) -e tu_prod
