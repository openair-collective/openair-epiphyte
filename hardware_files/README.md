# Hardware

## Bills of Materials (BOMs)

A BOM is a list of materials needed to reproduce the physical device as closely as possible. There are two BOM files:

- Sorbent Panel and CO<sub>2</sub> Sensor Box
- Electronics and HVAC

Each list includes information on obtaining the materials, including vendors, pricing, whether customization is needed.

In the sorbent panel BOM, there are also a `Item ID` column that corresponds to individual 3D CAD files. There are two tabs in this file--one that lists off-the-shelf components (IDs that start with `M`), and the other that lists custom components (IDs that start with `C`).

### CAD files for sorbent panel and CO<sub>2</sub> sensor box components

3D CAD files for individual components, either in `.step` or `.sldprt` format, are available for most line items. These files include those for off-the-shelf components (for convenience) and custom components (required for repeat fabrication by a machine shop). The CAD file names are preceded by the `Item ID` so that they can be matched up to the line item easily. The anatomy of the CAD file names is as follows:

`{Item ID}_{Vendor Name}_{Part Number}_{Part Description}.{Extension}`

For custom parts, the file name is simply:

`{Item ID}_{Part Description}.{Extension}`

Some custom parts are provided with multiple formats of fabrication files, including 2D and 3D CAD files. 2D files may come in two versions--with and without holes--in case one wants to drill holes themselves, e.g., to save cost or for hole size flexibility.

CAD files of all custom parts are accessible, at the time of the publication of this repository, in live-maintained [Onshape](https://www.onshape.com/) files. The BOM lists links for finding the Onshape version of each custom file. The assemblies can be found here: [sorbent panel](https://cad.onshape.com/documents/3e513bed349fbe5dac3c2097/w/f4ba0cf31e493613401f5693/e/31d268d611fa49f66a7aafab?renderMode=0&uiState=66065e0af10d7d54a1518440) and [sensor box](https://cad.onshape.com/documents/c282863293f77fd9e75ee4bd/w/2c50dc42efc36e588e65a721/e/b7f233adbe17413331fabfa8?renderMode=0&uiState=66065e2f9a780059827ee09f)

Note: Onshape is free to use for non-commercial/hobby purposes. The files are free to access and download, although they may need an Onshape account to fork (in the same way software repos are forked).

## Hardware Description
The Hardware Description folder includes high-level documentation of how the parts are connected, as well as pinouts and engineering notes for different classes of components.
