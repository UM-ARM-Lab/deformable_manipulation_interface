all: colab_folding cloth_table

colab_folding:
	$(eval EXPERIMENT = colab_folding)
	gs -q -sDEVICE=png256 -dEPSCrop -r300 -dNOPAUSE -dSAFER -dBATCH -sOutputFile=output_images/$(EXPERIMENT)/$(EXPERIMENT)__%03d.png output_images/$(EXPERIMENT)/*.eps
	cd output_images/$(EXPERIMENT) && \
	../../imToMov.sh 50 $(EXPERIMENT).wmv2 png
	mv output_images/$(EXPERIMENT)/$(EXPERIMENT).wmv2 output_images

cloth_table:
	$(eval EXPERIMENT = cloth_table)
	gs -q -sDEVICE=png256 -dEPSCrop -r300 -dNOPAUSE -dSAFER -dBATCH -sOutputFile=output_images/$(EXPERIMENT)/$(EXPERIMENT)__%03d.png output_images/$(EXPERIMENT)/*.eps
	cd output_images/$(EXPERIMENT) && \
	../../imToMov.sh 50 $(EXPERIMENT).wmv2 png
	mv output_images/$(EXPERIMENT)/$(EXPERIMENT).wmv2 output_images

rope_cylinder:
	$(eval EXPERIMENT = rope_cylinder)
	gs -q -sDEVICE=png256 -dEPSCrop -r300 -dNOPAUSE -dSAFER -dBATCH -sOutputFile=output_images/$(EXPERIMENT)/$(EXPERIMENT)__%03d.png output_images/$(EXPERIMENT)/*.eps
	cd output_images/$(EXPERIMENT) && \
	../../imToMov.sh 50 $(EXPERIMENT).wmv2 png
	mv output_images/$(EXPERIMENT)/$(EXPERIMENT).wmv2 output_images
