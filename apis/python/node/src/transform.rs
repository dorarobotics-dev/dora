use dora_transform::{Extrinsics, FrameDict, FrameError as CoreError, rigid};
use pyo3::create_exception;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use std::path::Path;

create_exception!(
    transform,
    FrameError,
    PyValueError,
    "A frame or calibration problem."
);
create_exception!(transform, UnknownFrameError, FrameError);
create_exception!(transform, MissingExtrinsicError, FrameError);
create_exception!(transform, DynamicEdgeError, FrameError);
create_exception!(transform, FrameOrderError, FrameError);

fn to_py(e: CoreError) -> PyErr {
    match e {
        CoreError::Invalid(m) => FrameError::new_err(m),
        CoreError::UnknownFrame(m) => UnknownFrameError::new_err(m),
        CoreError::MissingExtrinsic(m) => MissingExtrinsicError::new_err(m),
        CoreError::FrameOrder(m) => FrameOrderError::new_err(m),
        CoreError::DynamicEdge { message, edges } => {
            // str(e) is the message; the crossed edges ride along as `e.edges`.
            let err = DynamicEdgeError::new_err(message);
            Python::attach(|py| match err.value(py).setattr("edges", edges) {
                Ok(()) => err,
                Err(attach_error) => attach_error,
            })
        }
    }
}

#[pyclass(
    name = "FrameDict",
    module = "dora.transform",
    frozen,
    skip_from_py_object
)]
#[derive(Clone)]
pub struct PyFrameDict(pub FrameDict);

#[pymethods]
impl PyFrameDict {
    #[staticmethod]
    fn load(path: &str) -> PyResult<Self> {
        FrameDict::load(Path::new(path)).map(Self).map_err(to_py)
    }
    fn kind(&self, name: &str) -> PyResult<&'static str> {
        Ok(self.0.require(name).map_err(to_py)?.kind.as_str())
    }
    fn require(&self, name: &str) -> PyResult<()> {
        self.0.require(name).map(|_| ()).map_err(to_py)
    }
}

#[pyclass(
    name = "Extrinsics",
    module = "dora.transform",
    frozen,
    skip_from_py_object
)]
pub struct PyExtrinsics(pub Extrinsics);

#[pymethods]
impl PyExtrinsics {
    #[staticmethod]
    fn load(path: &str, frames: &PyFrameDict) -> PyResult<Self> {
        Extrinsics::load(Path::new(path), &frames.0)
            .map(Self)
            .map_err(to_py)
    }
    #[getter]
    fn frames(&self) -> PyFrameDict {
        PyFrameDict(self.0.frames().clone())
    }
    #[getter]
    fn path(&self) -> Option<String> {
        self.0.path().map(|p| p.display().to_string())
    }
}

#[pyclass(
    name = "Transform",
    module = "dora.transform",
    frozen,
    skip_from_py_object
)]
#[derive(Clone)]
pub struct PyTransform(pub rigid::Transform);

#[pymethods]
impl PyTransform {
    #[getter]
    fn src(&self) -> String {
        self.0.src().to_string()
    }
    #[getter]
    fn dst(&self) -> String {
        self.0.dst().to_string()
    }
    #[getter]
    fn matrix(&self) -> [[f64; 4]; 4] {
        *self.0.matrix()
    }
    fn then(&self, other: &PyTransform) -> PyResult<Self> {
        self.0.then(&other.0).map(Self).map_err(to_py)
    }
    fn inverse(&self) -> Self {
        Self(self.0.inverse())
    }
    fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        self.0.apply(p)
    }
    /// Packed little-endian float32 points, `item_step` floats each; returns new bytes.
    fn apply_packed<'py>(
        &self,
        py: Python<'py>,
        data: &[u8],
        item_step: usize,
    ) -> PyResult<Bound<'py, pyo3::types::PyBytes>> {
        if !data.len().is_multiple_of(4) {
            return Err(FrameError::new_err(
                "point bytes are not a whole number of float32",
            ));
        }
        let pts: Vec<f32> = data
            .chunks_exact(4)
            .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect();
        let out = self.0.apply_packed(&pts, item_step).map_err(to_py)?;
        let bytes: Vec<u8> = out.iter().flat_map(|v| v.to_le_bytes()).collect();
        Ok(pyo3::types::PyBytes::new(py, &bytes))
    }
}

#[pyfunction]
fn lookup(ex: &PyExtrinsics, src: &str, dst: &str) -> PyResult<PyTransform> {
    rigid::lookup(&ex.0, src, dst)
        .map(PyTransform)
        .map_err(to_py)
}

#[pyfunction]
fn pose_transform(
    values: Vec<f64>,
    parent_frame: &str,
    child_frame: &str,
    frames: &PyFrameDict,
) -> PyResult<PyTransform> {
    rigid::pose_transform(&values, parent_frame, child_frame, &frames.0)
        .map(PyTransform)
        .map_err(to_py)
}

#[pyfunction]
#[pyo3(signature = (ex, src, dst, pose=None))]
fn resolve(
    ex: &PyExtrinsics,
    src: &str,
    dst: &str,
    pose: Option<&PyTransform>,
) -> PyResult<(Option<PyTransform>, bool)> {
    let (t, used) = rigid::resolve(&ex.0, src, dst, pose.map(|p| &p.0)).map_err(to_py)?;
    Ok((t.map(PyTransform), used))
}

pub fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = parent.py();
    let m = PyModule::new(py, "transform")?;
    m.add_class::<PyFrameDict>()?;
    m.add_class::<PyExtrinsics>()?;
    m.add_class::<PyTransform>()?;
    m.add_function(wrap_pyfunction!(lookup, &m)?)?;
    m.add_function(wrap_pyfunction!(pose_transform, &m)?)?;
    m.add_function(wrap_pyfunction!(resolve, &m)?)?;
    m.add("FrameError", py.get_type::<FrameError>())?;
    m.add("UnknownFrameError", py.get_type::<UnknownFrameError>())?;
    m.add(
        "MissingExtrinsicError",
        py.get_type::<MissingExtrinsicError>(),
    )?;
    m.add("DynamicEdgeError", py.get_type::<DynamicEdgeError>())?;
    m.add("FrameOrderError", py.get_type::<FrameOrderError>())?;
    parent.add_submodule(&m)?;
    // `from dora import transform` needs the submodule in sys.modules.
    py.import("sys")?
        .getattr("modules")?
        .set_item("dora.transform", &m)?;
    Ok(())
}
