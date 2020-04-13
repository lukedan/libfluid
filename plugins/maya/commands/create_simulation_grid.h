#pragma once

/// \file
/// Declaration of the \p createSimulationGrid command.

#include <maya/MPxCommand.h>
#include <maya/MDGModifier.h>

namespace fluid::maya {
	/// A command that creates a simulation grid node and a mesher node.
	class create_simulation_grid_command : public MPxCommand {
	public:
		static const MString name; ///< The name of this command.

		/// Creates an instance of this command.
		static void *creator();

		/// Creates the nodes.
		MStatus doIt(const MArgList&) override;
		/// Undoes the operation.
		MStatus undoIt() override;
		/// Redoes the operation.
		MStatus redoIt() override;

		/// This command can be undone.
		bool isUndoable() const override {
			return true;
		}
	private:
		MDGModifier _graph_modifier; ///< Used to create nodes and links.
	};
}
