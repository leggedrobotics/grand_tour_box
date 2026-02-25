<p align="center" style="margin: 0;">
  <img
    src="box_documentation/images/2026_GRANDTOUR_LOGO.png"
    alt="GrandTour Banner"
    width="15%"
    style="display:block; margin:0 auto 6px auto;"
  />
</p>
<h1 align="center" style="margin: 0;">
  GrandTour - <i>Boxi</i> Software
</h1>

<p align="center">
  <em><small>A project brought to you by <a href="https://rsl.ethz.ch/">RSL - ETH Zurich</a>.</small></em>
</p>
<p align="center">
  <a href="#getting-started">Getting Started</a> •
  <a href="#citation">Citation</a> •
  <a href="#contributing">Contributing</a>
</p>

<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="ANYmal" style="position: relative; left: -60px;">
<h2 id="getting-started">
Getting Started
</h2>

This repository contains all drivers, real-time software, and post-processing tools required to operate Boxi, our multi-modal autonomy payload.

* 📖 More information, including setup instructions, will be available in the [Wiki](https://github.com/leggedrobotics/grand_tour_box/wiki) (🚧 under construction).
* 📦 The **GrandTour Dataset**, collected using Boxi, is available at: [https://grand-tour.leggedrobotics.com/](https://grand-tour.leggedrobotics.com/)
* 📄 The **GrandTour Dataset preprint** is available on arXiv: [https://arxiv.org/abs/2602.18164](https://arxiv.org/abs/2602.18164)
* 🔐 **Dataset Access / Formats**
  * Registration (ROS formats): [https://forms.gle/2qJkGYJ6oxnBvdNq9](https://forms.gle/2qJkGYJ6oxnBvdNq9)
  * Data overview: [https://grand-tour.leggedrobotics.com/dataset](https://grand-tour.leggedrobotics.com/dataset)
  * Hugging Face (ZARR): [https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset](https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset)
* 📈 The **Localization Benchmark** is available at: [https://grand-tour.leggedrobotics.com/tasks/localization](https://grand-tour.leggedrobotics.com/tasks/localization)
* 🔩 The **Hardware Specifications** are available at: [https://grand-tour.leggedrobotics.com/setup](https://grand-tour.leggedrobotics.com/setup)

> ⚙️ **Note:** We are currently in the process of refactoring the repository to improve usability. ROS1 → ROS2 converters have been released, and native ROS2 dataset support (MCAP) is coming soon.

<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="ANYmal" style="position: relative; left: -60px;">
<h2 id="citation">
Citation
</h2>

If you use this software or dataset, please cite our work:

### GrandTour Dataset Paper (arXiv preprint)

```bibtex
@misc{frey_tuna2026grandtour,
    title         = {GrandTour: A Legged Robotics Dataset in the Wild for Multi-Modal Perception and State Estimation},
    author        = {Jonas Frey and Turcan Tuna and Frank Fu and Katharine Patterson and Tianao Xu and Maurice Fallon and Cesar Cadena and Marco Hutter},
    year          = {2026},
    eprint        = {2602.18164},
    archivePrefix = {arXiv},
    primaryClass  = {cs.RO},
    url           = {https://arxiv.org/abs/2602.18164},
    note          = {\textsuperscript{*}Equal contribution (Turcan Tuna and Jonas Frey).}
}
```

### Boxi System Paper (RSS 2025)

```bibtex
@INPROCEEDINGS{Frey-Tuna-Fu-RSS-25,
    AUTHOR    = {Jonas Frey AND Turcan Tuna AND Lanke Frank Tarimo Fu AND Cedric Weibel AND Katharine Patterson AND Benjamin Krummenacher AND Matthias Müller AND Julian Nubert AND Maurice Fallon AND Cesar Cadena AND Marco Hutter},
    TITLE     = {{Boxi: Design Decisions in the Context of Algorithmic Performance for Robotics}},
    BOOKTITLE = {Proceedings of Robotics: Science and Systems},
    YEAR      = {2025},
    ADDRESS   = {Los Angeles, United States},
    MONTH     = {July},
    NOTE      = {\textsuperscript{*}Equal contribution (Jonas Frey and Turcan Tuna and Frank Fu).}
}
```

<img align="right" width="60" height="60" src="https://github.com/leggedrobotics/grand_tour_box/blob/main/box_documentation/images/icon.png" alt="ANYmal">
<h2 id="contributing">
Contributing
</h2>

We welcome contributions! If you're interested in contributing, please contact:

* Turcan Tuna – [tutuna@ethz.ch](mailto:tutuna@ethz.ch)
* Jonas Frey – [jonfrey@ethz.ch](mailto:jonfrey@ethz.ch)
