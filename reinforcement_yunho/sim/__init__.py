"""윤호's Isaac Sim data-generation package (spec §4 / §5 / EuRoC-ASL handoffs).

Everything importable/testable on a plain machine (numpy + pyyaml only). Anything
that needs Isaac Sim (``omni.replicator``) is import-guarded inside the modules, so
the pure logic — scene sampling, corner projection, dataset split, CSV/stream
writers — imports and unit-tests without a GPU. See ``sim/README.md`` for the
run-now-vs-needs-Isaac map and the exact teammate handoffs.

Modules:
    scene_gen          domain-randomised scene sampler (spec §4.1) + Isaac graph stub
    replicator_writer  pure build_label_lines() (§4.3) + omni.replicator Writer
    export_dataset     images/labels/{train,val,test} 80/10/10 + meta.jsonl (길남 schema)
    export_vio         EuRoC-ASL flight bag (mav0/...) for 태민 (VIO/OpenVINS)
    export_stream      §5 GT-pose stream for 태민, routed through 길남's gt_stream
    visualize_labels   eyeball a YOLO-pose label over its image (corner-order check)
"""
